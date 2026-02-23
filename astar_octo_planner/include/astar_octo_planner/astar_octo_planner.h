/*
 *  Copyright 2025, MASCOR
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
 *  authors:
 *    MASCOR
 *
 */

#ifndef OCTO_NAVIGATION__ASTAR_OCTO_PLANNER_H
#define OCTO_NAVIGATION__ASTAR_OCTO_PLANNER_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap/OcTree.h>
#include <unordered_map>
#include <unordered_set>
#include <mbf_octo_core/octo_planner.h>
#include <mbf_msgs/action/get_path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace astar_octo_planner
{

class AstarOctoPlanner : public mbf_octo_core::OctoPlanner
{
public:
  typedef std::shared_ptr<astar_octo_planner::AstarOctoPlanner> Ptr;

  AstarOctoPlanner();

  /**
   * @brief Destructor
   */
  virtual ~AstarOctoPlanner();

  /**
   * @brief Given a goal pose in the world, compute a plan
   *
   * @param start The start pose
   * @param goal The goal pose
   * @param tolerance If the goal is obstructed, how many meters the planner can
   * relax the constraint in x and y before failing
   * @param plan The plan... filled by the planner
   * @param cost The cost for the the plan
   * @param message Optional more detailed outcome as a string
   *
   * @return Result code as described on GetPath action result:
   *         SUCCESS         = 0
   *         1..9 are reserved as plugin specific non-error results
   *         FAILURE         = 50  # Unspecified failure, only used for old,
   * non-mfb_core based plugins CANCELED        = 51 INVALID_START   = 52
   *         INVALID_GOAL    = 53
   *         NO_PATH_FOUND   = 54
   *         PAT_EXCEEDED    = 55
   *         EMPTY_PATH      = 56
   *         TF_ERROR        = 57
   *         NOT_INITIALIZED = 58
   *         INVALID_PLUGIN  = 59
   *         INTERNAL_ERROR  = 60
   *         71..99 are reserved as plugin specific errors
   */
  virtual uint32_t makePlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal,
                            double tolerance, std::vector<geometry_msgs::msg::PoseStamped>& plan, double& cost,
                            std::string& message) override;

  /**
   * @brief Requests the planner to cancel, e.g. if it takes too much time.
   *
   * @return True if a cancel has been successfully requested, false if not
   * implemented.
   */
  virtual bool cancel() override;

  /**
   * @brief initializes this planner with the given plugin name and map
   *
   * @param name name of this plugin
   *
   * @return true if initialization was successul; else false
   */
  virtual bool initialize(const std::string& plugin_name, const rclcpp::Node::SharedPtr& node) override;


protected:
  /**
   * @brief runs dijkstra path planning and stores the resulting distances and predecessors to the fields potential and
   * predecessors of this class
   *
   * @param start[in] 3D starting position of the requested path
   * @param goal[in] 3D goal position of the requested path
   * @param path[out] optimal path from the given starting position to tie goal position
   *
   * @return result code in form of GetPath action result: SUCCESS, NO_PATH_FOUND, INVALID_START, INVALID_GOAL, and
   * CANCELED are possible
   */
  //uint32_t astar(const mesh_map::Vector& start, const mesh_map::Vector& goal, std::list<lvr2::VertexHandle>& path);

  /**
   * @brief runs dijkstra path planning
   *
   * @param start[in] 3D starting position of the requested path
   * @param goal[in] 3D goal position of the requested path
   * @param edge_weights[in] edge distances of the map
   * @param costs[in] vertex costs of the map
   * @param path[out] optimal path from the given starting position to tie goal position
   * @param distances[out] per vertex distances to goal
   * @param predecessors[out] dense predecessor map for all visited vertices
   *
   * @return result code in form of GetPath action result: SUCCESS, NO_PATH_FOUND, INVALID_START, INVALID_GOAL, and
   * CANCELED are possible
   */
  // uint32_t dijkstra(const mesh_map::Vector& start, const mesh_map::Vector& goal,
  //                   const lvr2::DenseEdgeMap<float>& edge_weights, const lvr2::DenseVertexMap<float>& costs,
  //                   std::list<lvr2::VertexHandle>& path, lvr2::DenseVertexMap<float>& distances,
  //                   lvr2::DenseVertexMap<lvr2::VertexHandle>& predecessors);


  /**
   * @brief gets called whenever the node's parameters change

   * @param parameters vector of changed parameters.
   *                   Note that this vector will also contain parameters not related to the dijkstra mesh planner.
   */
  rcl_interfaces::msg::SetParametersResult reconfigureCallback(std::vector<rclcpp::Parameter> parameters);

private:
  // // current map
  // mesh_map::MeshMap::Ptr mesh_map_;
  // name of this plugin
  std::string name_;
  // node handle
  rclcpp::Node::SharedPtr node_;
  // true if the abort of the current planning was requested; else false
  std::atomic_bool cancel_planning_;
  // publisher of resulting path
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  // publisher of z-lifted path for visualization (not occluded by voxels)
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr body_height_path_pub_;
  // tf frame of the map
  std::string map_frame_;
  // handle of callback for changing parameters dynamically
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr reconfiguration_callback_handle_;
  // config determined by ROS params; Init values defined here are used as default ROS param value
  struct {
    // publisher of resulting vector fiels
    bool publish_vector_field = false;
    // publisher of per face vectorfield
    bool publish_face_vectors = false;
    // offset of maximum distance from goal position
    double goal_dist_offset = 0.3;
    // defines the vertex cost limit with which it can be accessed
    double cost_limit = 1.0;
  } config_;

  // Utility functions used by the planner implementation.
  // Note: legacy nearest-3d helpers removed; graph-based planning uses nearest graph nodes instead.
  // (removed) findNodeByCoordinateInTree: legacy helper not used in current planner

  std::array<double, 3> gridToWorld(const std::tuple<int, int, int>& grid_pt);
  // (removed) hasNoOccupiedCellsAbove: legacy vertical-check helper not used in current planner
  bool isCylinderCollisionFree(const std::tuple<int, int, int>& coord, double radius);

  // Check that an occupied voxel has enough horizontal floor support at similar Z
  // to be a genuine floor/ramp surface (rejects thin wall tops).
  bool hasFloorSupport(double x, double y, double z, double node_size) const;

  // Check that a straight-line edge between two 3D points is collision-free
  // at robot body heights (samples multiple Z slices between ground and robot_height_).
  bool isEdgeCollisionFree(const octomap::point3d& from, const octomap::point3d& to) const;

  // Temporarily clear occupied voxels within a vertical cylinder around a point
  // center: world coordinates of cylinder center
  // radius: horizontal radius (meters)
  // z_bottom, z_top: vertical bounds (meters)
  void clearOccupiedCylinderAround(const geometry_msgs::msg::Point &center, double radius, double z_bottom, double z_top);

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;

  // Octree pointer (owned when we deserialize from messages)
  std::unique_ptr<octomap::OcTree> octree_;

  // Octomap / sensor model parameters
  double octomap_prob_hit_ = 0.7;
  double octomap_prob_miss_ = 0.3;
  double octomap_thres_ = 0.5;
  double octomap_clamp_min_ = 0.1;
  double octomap_clamp_max_ = 0.95;



  // Callback for octomap subscription
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);

  // Occupancy grid removed: planner relies on octree as authoritative map source.

  // Grid sizes when using an octree-backed grid
  int grid_size_x_ = 0;
  int grid_size_y_ = 0;
  int grid_size_z_ = 0;

  // Active voxel size used for grid/world conversions (set from octree resolution)
  double active_voxel_size_ = 0.1;

  // (legacy search-limiting members removed)

  // Voxel grid parameters.
  double voxel_size_;
  double z_threshold_;
  // robot footprint and collision params (meters)
  double robot_radius_ = 0.35;
  double robot_width_ = 0.4;   // default width (m)
  double robot_length_ = 1.5;  // default length (m)
  double robot_height_ = 0.7;  // default height (m)
  double footprint_margin_ = 0.05; // safety margin around footprint (m)
  // sampling density for footprint collision checks
  int footprint_samples_x_ = 3;
  int footprint_samples_y_ = 3;
  double min_vertical_clearance_ = -0.5;
  double max_vertical_clearance_ = 0.6;
  // Floor support: minimum fraction of ring directions that must have occupied
  // neighbors at similar Z for a node to be considered walkable (rejects wall tops)
  double min_floor_support_ratio_ = 0.4;
  int floor_support_num_dirs_ = 8;  // number of directions to sample in the ring
  // Max vertical distance from a surface (occupied cell) a free node may be to be included
  double max_surface_distance_ = 0.25;
  // Maximum step/climb height (meters) allowed between node and surface for acceptance
  double max_step_height_ = 0.30;
  // Stair-edge augmentation controls (search for reachable landings above current node)
  bool enable_stair_edges_ = true;
  double stair_xy_radius_ = 0.15;
  double stair_vertical_margin_ = 0.05;

  // Minimum bound for the occupancy grid.
  std::array<double, 3> min_bound_;
  // Maximum bound for the occupancy grid.
  std::array<double, 3> max_bound_;

  // Corner/edge penalty tuning (defaults increased to keep robot away from walls)
  double corner_radius_ = 0.20;           // meters
  double corner_penalty_weight_ = 5.0;    // multiplier for penalty
  // RANSAC-based wall/corner detection
  double ransac_radius_ = 2.0;           // meters, neighborhood search radius
  double ransac_dist_thresh_ = 0.1;      // meters, inlier distance threshold
  int ransac_min_inliers_ = 16;            // minimum inliers to accept a line
  int ransac_iterations_ = 30;            // RANSAC iterations
  double wall_penalty_weight_ = 2.0;      // weight applied for wall-border nodes
  double corner_angle_thresh_deg_ = 45.0; // degrees threshold to consider two lines a corner
  // Sector-histogram detector params
  int sector_bins_ = 8;                  // number of angular sectors
  double sector_radius_ = 0.40;           // neighbor radius for sectors (m)
  double sector_peak_thresh_ = 0.55;      // fraction of neighbors in peak bin to consider wall
  int sector_min_inliers_ = 4;            // minimum neighbors to run sector logic

  // Centroid-shift (mean-shift inspired) edge detector parameters
  int centroid_k_ = 12;                 // k-nearest neighbors to use for centroid shift
  double centroid_lambda_ = 1.0;        // lambda multiplier for local resolution threshold
  double centroid_penalty_weight_ = 2.0;// penalty weight applied when centroid shift indicates edge

  // Penalty spread
  double penalty_spread_radius_ = 4.5; // meters, radius within which to spread a fraction of the penalty
  double penalty_spread_factor_ = 1.0; // fraction [0..1] of the penalty to distribute among neighbors
  int worker_thread_limit_ = 0; // 0 => auto (use hardware concurrency)

  // Hash function for tuple<int, int, int> to track unique occupied voxels
  struct TupleHash {
    template <typename T1, typename T2, typename T3>
    std::size_t operator()(const std::tuple<T1, T2, T3>& t) const {
      auto [a, b, c] = t;
      return std::hash<T1>()(a) ^ std::hash<T2>()(b) ^ std::hash<T3>()(c);
    }
  };

  // --- Prototype graph representation (coarse empty-space graph) ---
  struct GraphNode {
    octomap::OcTreeKey key;
    unsigned int depth = 0;
    octomap::point3d center;
    double size = 0.0;
    bool is_walkable = true;  // true = floor surface with clearance, false = wall/obstacle
    std::string id() const {
      char buf[128];
      std::snprintf(buf, sizeof(buf), "%u_%u_%u_%u", key.k[0], key.k[1], key.k[2], depth);
      return std::string(buf);
    }
  };

  // --- Double-buffered graph data structure ---
  // All graph data is bundled into this struct, allowing atomic swap between
  // active (used for planning) and pending (being built in background) buffers.
  struct GraphData {
    std::unordered_map<std::string, GraphNode> nodes;
    std::unordered_map<std::string, std::vector<std::string>> adj;
    std::unordered_map<std::string, double> node_penalty;
    std::unordered_set<std::string> penalized_nodes;
    // Incremental build tracking (only used during build, not during planning)
    std::unordered_set<std::string> processed_occupied_keys;
    std::unordered_set<std::string> nodes_needing_adjacency_update;

    bool empty() const { return nodes.empty(); }
    size_t size() const { return nodes.size(); }
  };

  // Active graph: always available for planning (never null after first build)
  std::shared_ptr<GraphData> active_graph_;
  // Mutex protecting active_graph_ pointer swap
  std::mutex graph_mutex_;

  // Legacy direct-access members kept for compatibility with existing code paths
  // (will be migrated to active_graph_ usage over time)
  std::unordered_map<std::string, GraphNode> graph_nodes_;
  std::unordered_map<std::string, std::vector<std::string>> graph_adj_;
  std::unordered_map<std::string, double> graph_node_penalty_;
  std::unordered_set<std::string> graph_penalized_nodes_;

  // Background graph builder members
  rclcpp::TimerBase::SharedPtr graph_build_timer_;
  std::atomic_bool graph_dirty_{false};
  std::atomic_bool graph_building_{false}; // True while background build is in progress
  std::atomic_bool penalties_dirty_{true}; // True when penalties need recomputing (on next plan request)
  // Marker publisher for graph visualization
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr graph_marker_pub_;
  bool publish_graph_markers_ = true;
  // When false, incoming octomap messages are ignored and the background
  // graph rebuild timer will not rebuild the graph. Subscription stays active.
  bool enable_octomap_updates_ = true;
  // When true, merge new octomap data incrementally instead of replacing.
  // This allows building the graph continuously as the robot explores.
  bool incremental_graph_build_ = true;

  // Octomap topic name (configurable)
  std::string octomap_topic_ = std::string("/navigation/octomap_binary");

  // Legacy incremental tracking members (now inside GraphData, kept for transition)
  std::unordered_set<std::string> processed_occupied_keys_;
  std::unordered_set<std::string> nodes_needing_adjacency_update_;

  // Build a sampling-based connectivity graph over interior empty nodes.
  // eps: small epsilon distance (meters) to sample just outside node boundaries.
  // If target_graph is provided, builds into that graph; otherwise builds into legacy members.
  void buildConnectivityGraph(double eps = 0.05);
  void buildConnectivityGraphInto(std::shared_ptr<GraphData>& target, double eps = 0.05);

  // Incrementally update the connectivity graph with new voxels.
  // Only processes new occupied voxels and updates affected adjacencies.
  void updateConnectivityGraphIncremental(double eps = 0.05);
  void updateConnectivityGraphIncrementalInto(std::shared_ptr<GraphData>& target, double eps = 0.05);

  // Publish graph nodes as visualization Markers (CUBE_LIST)
  void publishGraphMarkers();
  void publishGraphMarkers(const std::shared_ptr<GraphData>& graph);

  // Find the closest graph node id to a world coordinate (returns empty if none)
  std::string findClosestGraphNode(const octomap::point3d& p) const;
  std::string findClosestGraphNode(const octomap::point3d& p, const std::shared_ptr<GraphData>& graph) const;

  // Plan on the built graph using a simple A* and return a vector of node ids.
  std::vector<std::string> planOnGraph(const std::string& start_id, const std::string& goal_id) const;

  // TF and origin-shift stuff removed; this planner requires start/goal to be
  // provided in the octomap's map frame (map_frame_).
};

}  // namespace astar_octo_planner

#endif  // OCTO_NAVIGATION__ASTAR_OCTO_PLANNER_H
