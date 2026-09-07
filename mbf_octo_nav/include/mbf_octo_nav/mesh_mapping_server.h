/*
 *  Copyright 2025, Mascor Institute
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  author: Mascor Institute
 */

#ifndef MBF_OCTO_NAV__MESH_MAPPING_SERVER_H
#define MBF_OCTO_NAV__MESH_MAPPING_SERVER_H

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <mesh_msgs/msg/mesh_geometry_stamped.hpp>
#include <mesh_msgs/msg/mesh_vertex_costs_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <mbf_octo_core/mesh_mapper.h>

namespace mbf_octo_nav
{

/**
 * @brief Converts occupied octomap voxels into a persistent, low-poly triangle
 *        mesh, published as mesh_msgs::msg::MeshGeometryStamped on ~mesh_topic
 *        (default /meshmap), plus a per-vertex traversal cost layer published
 *        as mesh_msgs::msg::MeshVertexCostsStamped on ~costs_topic (default
 *        /meshmap_costs).
 *
 * The source octomap topic (see OctoMappingServer, default
 * /octomap_binary_local) only carries a local rolling window around the robot
 * and drops voxels once they fall outside of it. MeshMappingServer keeps every
 * voxel it has ever seen in its own append-only occupancy grid
 * (occupied_bins_), so the published mesh only ever grows even as the source
 * octomap shrinks.
 *
 * Two update paths:
 *  - octomapCallback(): cheap incremental path. Bins new voxels at
 *    mesh_voxel_size_ (independent of, and normally coarser than, the octomap
 *    resolution) and greedy-meshes only the newly-seen bins (face culling +
 *    rectangle merging), appending the result to the live mesh.
 *  - optimizeMesh(): fired every optimize_period_ seconds. Rebuilds the whole
 *    mesh from occupied_bins_ from scratch, which lets greedy meshing merge
 *    faces across old incremental-update seams into bigger, smoother
 *    rectangles. Immediately after, it classifies every vertex (wall / floor
 *    incline / stair riser / edge / corner / mesh border — see
 *    computeVertexCosts(), classifyRealStaircases(), computeFrontierVertices())
 *    and publishes the resulting cost layer.
 *
 * getReadyMesh() / getReadyVertexCosts() hand out a deep-copy snapshot of the
 * live mesh/costs under a short lock, mirroring OctoMappingServer::
 * getReadyGraph() — a consumer can call these at any time and always get a
 * consistent, ready-to-use copy.
 *
 * Implements mbf_octo_core::MeshMapper (getReadyMeshGraph()) so it can be
 * injected into mesh planner plugins the same way OctoMappingServer is
 * injected into OctoPlanner plugins. In addition to the mesh + costs,
 * getReadyMeshGraph() bundles the vertex graph Gv (vertex_adjacency_) and
 * triangle graph Gt (face_adjacency_ + face_normals_) from Wiemann et al.'s
 * Graph Half Edge Mesh — built once per optimizeMesh() pass in
 * buildGraphTopology() — so a planner can run Dijkstra/potential-field search
 * and reconstruct a per-triangle vector field without owning any mesh state.
 */
class MeshMappingServer : public mbf_octo_core::MeshMapper
{
public:
  using Ptr = std::shared_ptr<MeshMappingServer>;

  MeshMappingServer();
  ~MeshMappingServer() = default;

  /**
   * @brief Initialise the server: declare ROS params, subscribe to the octomap
   *        topic, advertise the mesh + vertex-costs topics, start the
   *        periodic optimize timer.
   * @param name  Parameter namespace prefix (e.g. "mesh_mapping_server").
   * @param node  Shared ROS node.
   */
  void initialize(const std::string & name, const rclcpp::Node::SharedPtr & node);

  //! Deep-copy snapshot of the current mesh — safe to hand to a planner.
  std::shared_ptr<mesh_msgs::msg::MeshGeometryStamped> getReadyMesh() const;
  //! Deep-copy snapshot of the current per-vertex cost layer.
  std::shared_ptr<mesh_msgs::msg::MeshVertexCostsStamped> getReadyVertexCosts() const;

  // ---- mbf_octo_core::MeshMapper interface --------------------------------
  std::shared_ptr<mbf_octo_core::MeshGraphData> getReadyMeshGraph() override;
  std::string getMapFrame() const override;
  void publishAdditionalMarkers(const visualization_msgs::msg::MarkerArray & ma) override;

private:
  // Integer lattice coordinate, in units of mesh_voxel_size_. Used both for
  // occupancy bins and for mesh corner vertices (bins and corners share the
  // same lattice: corner (x,y,z) is bin (x,y,z)'s "min" corner).
  using Bin = std::array<int32_t, 3>;
  struct BinHash
  {
    size_t operator()(const Bin & k) const noexcept
    {
      size_t h = std::hash<int32_t>()(k[0]);
      h = h * 73856093u ^ std::hash<int32_t>()(k[1]);
      h = h * 19349663u ^ std::hash<int32_t>()(k[2]);
      return h;
    }
  };
  struct XYHash
  {
    size_t operator()(const std::pair<int32_t, int32_t> & k) const noexcept
    {
      return std::hash<int64_t>()((static_cast<int64_t>(k.first) << 32) ^
                                   static_cast<uint32_t>(k.second));
    }
  };

  // Per-vertex bookkeeping accumulated while greedy-meshing, consumed by
  // computeVertexCosts(). Directions: 0=+x,1=-x,2=+y,3=-y,4=+z,5=-z.
  struct VertexFeatures
  {
    std::array<bool, 6> dirs{};
    // touches a wall-normal (x/y) face whose vertical extent is <=
    // stair_riser_max_height_ (a stair riser), vs. a genuine tall wall.
    bool any_short_vertical_face = false;
    bool any_tall_vertical_face  = false;
  };

  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);

  // Periodic (every optimize_period_ s): full remesh from occupied_bins_ (lets
  // greedy merging combine faces across old incremental seams into bigger,
  // smoother ones), then corner/edge/slope classification + cost publish.
  void optimizeMesh();

  // Greedy-mesh exactly the bins in `bins` (assumed already inserted into
  // occupied_bins_) and append the resulting quads to mesh_msg_, updating
  // vertex_features_ / floor_top_height_ along the way.
  void meshBins(const std::unordered_set<Bin, BinHash> & bins);

  // Look up (or create) the vertex index for a corner lattice point, appending
  // a new vertex/normal/feature entry on first use.
  uint32_t vertexIndexFor(const Bin & corner);

  bool isOccupied(const Bin & bin) const;

  // Rebuilds costs_msg_ from vertex_features_ / floor_top_height_. Must run
  // immediately after a meshBins() pass so vertex indices line up.
  void computeVertexCosts();

  // Pools floor_top_height_ into smoothed_floor_height_: average floor height
  // per slope_smoothing_radius_-sized (x,y) cell. Raw per-bin heights are far
  // too noisy for slope estimation on textured surfaces (e.g. a ramp built
  // from evenly-spaced wooden slats reads, bin-for-bin, like a staircase);
  // averaging over a region wider than the texture period recovers the
  // underlying incline. Must run before floorSlopeCost() is used.
  void buildSmoothedFloorHeights();

  // Cost in [floor_flat_cost_, wall_cost_], increasing with the steepest
  // smoothed height step to a horizontally-adjacent region (see
  // buildSmoothedFloorHeights()) — a proxy for incline/decline steepness that
  // is robust to sub-cell surface texture.
  float floorSlopeCost(const Bin & corner) const;

  // Distinguishes real multi-step staircases from isolated short bumps (a
  // curb, a rock, a cable duct — anything short enough to look like a stair
  // riser but not actually part of one). Candidates are every vertex with
  // any_short_vertical_face set; candidates within stair_max_xy_dist_ of each
  // other are grouped (union-find); a group only counts as a real staircase
  // once it reaches stair_min_chain_length_ members. Vertices in smaller
  // groups are NOT flagged here, so computeVertexCosts() falls back to
  // treating them as ordinary obstacles instead of cheap stairs.
  std::vector<bool> classifyRealStaircases() const;

  // True frontier detection, deliberately NOT based on triangle-edge topology
  // (an edge used by only 1 triangle): T-junctions — very common with
  // max_triangle_edge_ capping quad size — create that exact same signature
  // throughout the mesh interior wherever differently-sized merged quads
  // meet, not just at the real edge of the mapped area. A small cost bump
  // there is invisible, but once border_cost_boost_ is large enough to seed
  // inflation, treating every T-junction seam as a "frontier" chokes off
  // most of the room. Ground truth instead: for each floor-top vertex, look
  // frontier_check_radius_ out in 8 directions in occupied_bins_ (the raw
  // voxel occupancy) — genuinely nothing there in any direction means this
  // really is the edge of explored space.
  std::vector<bool> computeFrontierVertices() const;

  // Multi-source Dijkstra spread over the mesh's vertex/edge graph, mirroring
  // OctoMappingServer's graph-border penalty spread: every vertex whose
  // intrinsic cost is >= inflation_seed_threshold_ (walls, wall/floor edges,
  // corners) seeds a wave that decays linearly to 0 over inflation_radius_.
  // Final cost = max(intrinsic, spread) — this is what turns a hard
  // wall-vs-floor step into a smooth gradient, so a cost-minimizing planner
  // is pulled toward the centerline of a corridor/ramp instead of merely
  // avoiding the walls themselves. Must run after computeVertexCosts().
  void inflateCostsFromHazards();

  // Builds the vertex graph Gv (vertex_adjacency_) and triangle graph Gt
  // (face_adjacency_, face_normals_) from the freshly rebuilt mesh_msg_.
  // Gv is also reused by inflateCostsFromHazards() instead of it rebuilding
  // its own copy. Must run right after meshBins(occupied_bins_), before
  // getReadyMeshGraph() is called.
  void buildGraphTopology();

  // Octomap voxelization turns a continuous ramp into a literal staircase of
  // tiny flat treads + vertical risers. This relaxes (Laplacian-smooths) the
  // Z height of "step-edge" vertices — floor-top vertices that also touch a
  // short (<= max_ramp_rise_) vertical riser — toward their floor-neighbors,
  // several iterations, so a run of small steps eases into a slope instead of
  // staying a hard step. Transitions taller than max_ramp_rise_ (real stairs,
  // walls) are never touched. Only moves vertex Z (topology/triangle count is
  // unchanged); needs vertex_adjacency_ (Gv) from buildGraphTopology() first,
  // and buildGraphTopology() must run again afterward so face_normals_
  // reflect the smoothed positions.
  void smoothRampTransitions();

  // A mesh edge is manifold if it's used by exactly 1 (boundary — the edge of
  // what's been mapped, or a genuine T-junction gap; normal, left alone) or 2
  // (interior) triangles. Used by 3+ is non-manifold and gets repaired here:
  // for each such edge, the first 2 triangles that claim it (in face order)
  // are kept and every triangle after that is dropped — greedy but simple,
  // and safe (a dropped triangle just leaves a small boundary gap where it
  // was, never breaks a *different* edge's manifold-ness). Only ever needed
  // if some other geometry-editing pass merged/moved vertices without
  // understanding topology; the base greedy mesher in meshBins() can't
  // produce a non-manifold edge on its own. Must run right after meshBins(),
  // before buildGraphTopology() assumes at most 2 owners per edge.
  void enforceManifoldTriangles();

  // Fills clearance_above_ / clearance_lateral_ for every vertex, straight
  // from occupied_bins_ (the raw voxel occupancy) — this is why the check
  // lives in the mapper rather than the planner: only the mapper has that
  // data. Vertical: consecutive free bins straight up before hitting
  // occupied space. Lateral: isotropic — the smallest distance to occupied
  // space found scanning clearance_scan_num_dirs_ directions outward at the
  // vertex's own height. Both capped at their respective clearance_scan_max_*.
  void computeClearances();
  float computeClearanceAbove(const Bin & corner) const;
  float computeClearanceLateral(const Bin & corner) const;

  // ---- ROS handles -----------------------------------------------------
  rclcpp::Node::SharedPtr node_;
  std::string name_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Publisher<mesh_msgs::msg::MeshGeometryStamped>::SharedPtr mesh_pub_;
  rclcpp::Publisher<mesh_msgs::msg::MeshVertexCostsStamped>::SharedPtr costs_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr optimize_timer_;

  // ---- Parameters --------------------------------------------------------
  // Master switch: if false, initialize() returns immediately after this is
  // read — no subscriptions, timers, or publishers are ever created. Lets you
  // fully turn this mapper off from the yaml (e.g. to test OctoMappingServer
  // in isolation) without touching code.
  bool enabled_ = true;

  std::string octomap_topic_ = "/octomap_binary_local";
  std::string mesh_topic_    = "/meshmap";
  std::string costs_topic_   = "/meshmap_costs";
  std::string costs_layer_name_ = "traversal_cost";
  // Bin size used for meshing. Independent of the octomap resolution — set
  // larger than it to further reduce polygon detail on flat walls/floors.
  // Must stay <= stair_riser_max_height_ or no vertical face can ever
  // resolve as "short" (see the sanity check logged in initialize()).
  double mesh_voxel_size_    = 0.15;
  // Caps the greedy merge: no quad grows wider or taller than this (metres),
  // so one big flat wall/floor doesn't collapse into a single giant
  // rectangle. The merged quad's diagonal (from triangulating it) can still
  // exceed this by up to sqrt(2)x — that's inherent to any quad->triangle
  // mesh, not a case this cap is meant to cover.
  double max_triangle_edge_ = 0.5;
  // Vertical transitions at or below this (metres) get smoothed into a
  // continuous ramp by smoothRampTransitions() instead of staying a discrete
  // step. Taller transitions are left as a real step/wall.
  double max_ramp_rise_ = 0.2;
  // Relaxation iterations for smoothRampTransitions(). More iterations
  // spread a run of small steps into a gentler, longer slope.
  int ramp_smoothing_iterations_ = 4;
  // Period of the optimize timer tick. Each tick only actually runs the
  // full-remesh + cost pass if at least min_new_bins_for_optimize_ bins have
  // been added since the last run (see bins_at_last_optimize_) — otherwise
  // it's a no-op, so a stalled/slow-growing map doesn't pay for a full
  // remesh with nothing new to show for it.
  double optimize_period_    = 3.0;
  int min_new_bins_for_optimize_ = 3000;
  // Wall-normal faces with vertical extent at or below this (meters) are
  // treated as stair risers rather than walls.
  double stair_riser_max_height_ = 0.22;
  // Radius (meters) over which floor heights are averaged before estimating
  // slope. Should be a few times wider than any surface texture (e.g. ramp
  // slats) you want to ignore, but well under the scale of real floor-level
  // changes you want to detect.
  double slope_smoothing_radius_ = 0.5;
  // Max horizontal distance (meters) between short-riser candidates for them
  // to be considered part of the same staircase run.
  double stair_max_xy_dist_ = 0.45;
  // Minimum number of chained short-riser candidates required to count as a
  // real staircase (see classifyRealStaircases()). Below this, they're
  // treated as an isolated obstacle instead.
  int stair_min_chain_length_ = 3;

  // Robot-footprint collision scan (see computeClearances()). Caps bound how
  // far the scans search — set them comfortably above your tallest/widest
  // robot so a genuinely-clear vertex isn't capped down to a smaller value.
  double clearance_scan_max_height_ = 2.0;
  double clearance_scan_max_radius_ = 1.0;
  int clearance_scan_num_dirs_ = 8;
  // Height above a vertex's own level (metres) that the collision-check
  // reference point sits at, for BOTH clearance checks — never evaluated
  // directly on the floor surface itself:
  //  - Lateral: scans for obstacles at this height instead of at the
  //    vertex's exact level. At floor level, a neighboring floor patch just
  //    one voxel higher (voxel noise, or the ramp/stair texture) registers
  //    as "occupied" and produces false walls everywhere; scanning at
  //    roughly mid-body height instead finds genuine walls (which extend up
  //    through that height) while floor bumps don't reach it. Same trick as
  //    OctoMappingServer's wall_proximity_height_.
  //  - Vertical: the band from the floor up to this height is treated as
  //    always-clear and never occupancy-checked at all — that's leg/foot
  //    space for a legged robot (or just ground-level texture noise for any
  //    robot), not where the body itself needs clearance.
  double collision_check_height_offset_ = 0.5;

  // A newly-seen occupied leaf needs at least this many occupied 6-connected
  // neighbors (in the just-decoded octree) to be meshed at all; below that
  // it's treated as sensor noise and dropped, instead of punching a
  // face-culling hole in whatever surface is next to it. 0 disables the
  // filter entirely.
  int noise_filter_min_neighbors_ = 1;

  // Two floor-top vertices that are lattice-adjacent (1 bin apart in x or y)
  // but weren't connected by a shared triangle edge — which happens at every
  // T-junction, where a big greedy-merged quad's edge isn't split to match a
  // smaller neighboring quad's corner — get an extra Gv edge bridged between
  // them in buildGraphTopology(), as long as their height difference is
  // within this (metres). Without this, Dijkstra sees a fake disconnection
  // at every such seam even though the mesh visually touches there.
  double lattice_bridge_max_dz_ = 0.3;
  // How far (metres) computeFrontierVertices() looks in each of 8 directions
  // for any occupied floor before concluding there's genuinely nothing
  // there. Too small and normal mesh-detail gaps between occupied bins could
  // register as false frontier; too large and the true frontier only gets
  // flagged well short of the actual edge.
  double frontier_check_radius_ = 0.4;

  // Cost layer tuning, all in [0, 1] (clamped). See computeVertexCosts().
  double wall_cost_         = 0.8;
  double stair_cost_        = 0.25;
  double floor_flat_cost_   = 0.05;
  double edge_cost_boost_   = 0.15;
  double corner_cost_boost_ = 0.05;
  // Deliberately large enough that even a border vertex on plain flat floor
  // (floor_flat_cost_ + this) crosses inflation_seed_threshold_ below — a
  // small bump on the exact border vertices alone doesn't create any buffer
  // zone approaching the frontier; the point is for borders to seed the same
  // cost-inflation spread walls/corners already get, so the planner is
  // nudged away from hugging the edge of the mapped area before it actually
  // gets there, not just penalized once it's already at the dead end.
  double border_cost_boost_ = 0.5;

  // Cost inflation (spread from hazards — see inflateCostsFromHazards()).
  // Radius should span at least half the width of the corridors/ramps you
  // want the planner centered on: with walls on both sides closer together
  // than 2x this radius, the two spreads overlap and create a valley whose
  // minimum sits at the midline.
  double inflation_radius_ = 1.2;
  double inflation_factor_ = 1.0;
  double inflation_seed_threshold_ = 0.5;

  std::string map_frame_ = "map";

  // ---- State ---------------------------------------------------------------
  // Append-only: every occupied bin ever seen, regardless of whether the
  // source octomap still carries it (it may have scrolled out of the local
  // window).
  std::unordered_set<Bin, BinHash> occupied_bins_;
  // occupied_bins_.size() as of the last optimize pass that actually ran (see
  // min_new_bins_for_optimize_).
  size_t bins_at_last_optimize_ = 0;
  // Corner lattice point -> vertex index in mesh_msg_.mesh_geometry.vertices.
  std::unordered_map<Bin, uint32_t, BinHash> vertex_index_;
  // Reverse of vertex_index_: vertex index -> corner lattice point.
  std::vector<Bin> vertex_corners_;
  // Reverse of vertex_index_: vertex index -> accumulated face-touch features.
  std::vector<VertexFeatures> vertex_features_;
  // (bin_x, bin_y) -> z of a +Z (floor-top) face seen there. Sparse: only
  // populated at quad corners, which is exactly where floor height actually
  // changes (interior of one merged flat quad has nothing to look up, and
  // needs none — it's flat by construction).
  std::unordered_map<std::pair<int32_t, int32_t>, int32_t, XYHash> floor_top_height_;
  // (cell_x, cell_y) in slope_smoothing_radius_ units -> average floor height
  // (meters) of all floor_top_height_ samples in that cell. Rebuilt by
  // buildSmoothedFloorHeights() each optimize pass.
  std::unordered_map<std::pair<int32_t, int32_t>, double, XYHash> smoothed_floor_height_;

  // ---- Graph Half Edge Mesh topology (built by buildGraphTopology()) -------
  // Gv: per-vertex list of (neighbor vertex index, edge length).
  std::vector<std::vector<mbf_octo_core::MeshGraphData::VertexEdge>> vertex_adjacency_;
  // Gt: per-face up to 3 neighbor face indices (-1 = boundary edge).
  std::vector<std::array<int32_t, 3>> face_adjacency_;
  // Per-face unit normal.
  std::vector<std::array<float, 3>> face_normals_;

  // ---- Robot-footprint collision data (built by computeClearances()) -------
  std::vector<float> clearance_above_;
  std::vector<float> clearance_lateral_;

  mesh_msgs::msg::MeshGeometryStamped mesh_msg_;
  mesh_msgs::msg::MeshVertexCostsStamped costs_msg_;
  mutable std::mutex mesh_mutex_;
};

}  // namespace mbf_octo_nav

#endif  // MBF_OCTO_NAV__MESH_MAPPING_SERVER_H
