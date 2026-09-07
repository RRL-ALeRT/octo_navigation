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

#include "mbf_octo_nav/mesh_mapping_server.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <vector>

#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <geometry_msgs/msg/point.hpp>
#include <mesh_msgs/msg/mesh_triangle_indices.hpp>

namespace mbf_octo_nav
{

// =============================================================================
// Constructor
// =============================================================================

MeshMappingServer::MeshMappingServer()
{
}

// =============================================================================
// initialize
// =============================================================================

void MeshMappingServer::initialize(const std::string & name,
                                    const rclcpp::Node::SharedPtr & node)
{
  name_ = name;
  node_ = node;

  enabled_ = node_->declare_parameter(name_ + ".enabled", enabled_);
  if (!enabled_) {
    RCLCPP_INFO(node_->get_logger(),
      "MeshMappingServer (name=%s) disabled via '%s.enabled' — no subscriptions, "
      "no mesh will ever be built.",
      name_.c_str(), name_.c_str());
    return;
  }

  octomap_topic_   = node_->declare_parameter(name_ + ".octomap_topic",   octomap_topic_);
  mesh_topic_      = node_->declare_parameter(name_ + ".mesh_topic",      mesh_topic_);
  costs_topic_     = node_->declare_parameter(name_ + ".costs_topic",     costs_topic_);
  costs_layer_name_= node_->declare_parameter(name_ + ".costs_layer_name",costs_layer_name_);
  mesh_voxel_size_ = node_->declare_parameter(name_ + ".mesh_voxel_size", mesh_voxel_size_);
  max_triangle_edge_ = node_->declare_parameter(name_ + ".max_triangle_edge", max_triangle_edge_);
  max_ramp_rise_ = node_->declare_parameter(name_ + ".max_ramp_rise", max_ramp_rise_);
  ramp_smoothing_iterations_ = node_->declare_parameter(
    name_ + ".ramp_smoothing_iterations", ramp_smoothing_iterations_);
  optimize_period_ = node_->declare_parameter(name_ + ".optimize_period", optimize_period_);
  min_new_bins_for_optimize_ = node_->declare_parameter(
    name_ + ".min_new_bins_for_optimize", min_new_bins_for_optimize_);
  stair_riser_max_height_ = node_->declare_parameter(
    name_ + ".stair_riser_max_height", stair_riser_max_height_);
  slope_smoothing_radius_ = node_->declare_parameter(
    name_ + ".slope_smoothing_radius", slope_smoothing_radius_);
  stair_max_xy_dist_ = node_->declare_parameter(name_ + ".stair_max_xy_dist", stair_max_xy_dist_);
  stair_min_chain_length_ = node_->declare_parameter(
    name_ + ".stair_min_chain_length", stair_min_chain_length_);

  clearance_scan_max_height_ = node_->declare_parameter(
    name_ + ".clearance_scan_max_height", clearance_scan_max_height_);
  clearance_scan_max_radius_ = node_->declare_parameter(
    name_ + ".clearance_scan_max_radius", clearance_scan_max_radius_);
  clearance_scan_num_dirs_ = node_->declare_parameter(
    name_ + ".clearance_scan_num_dirs", clearance_scan_num_dirs_);
  collision_check_height_offset_ = node_->declare_parameter(
    name_ + ".collision_check_height_offset", collision_check_height_offset_);
  noise_filter_min_neighbors_ = node_->declare_parameter(
    name_ + ".noise_filter_min_neighbors", noise_filter_min_neighbors_);
  lattice_bridge_max_dz_ = node_->declare_parameter(
    name_ + ".lattice_bridge_max_dz", lattice_bridge_max_dz_);
  frontier_check_radius_ = node_->declare_parameter(
    name_ + ".frontier_check_radius", frontier_check_radius_);

  if (stair_riser_max_height_ < mesh_voxel_size_) {
    RCLCPP_WARN(node_->get_logger(),
      "MeshMappingServer: stair_riser_max_height (%.3f) < mesh_voxel_size (%.3f) — "
      "the smallest possible vertical face is one bin tall, so it can never "
      "resolve as a stair riser at this bin size. Walls and stair/ramp texture "
      "will be classified identically. Lower mesh_voxel_size or raise "
      "stair_riser_max_height.",
      stair_riser_max_height_, mesh_voxel_size_);
  }

  wall_cost_         = node_->declare_parameter(name_ + ".wall_cost",         wall_cost_);
  stair_cost_        = node_->declare_parameter(name_ + ".stair_cost",        stair_cost_);
  floor_flat_cost_   = node_->declare_parameter(name_ + ".floor_flat_cost",   floor_flat_cost_);
  edge_cost_boost_   = node_->declare_parameter(name_ + ".edge_cost_boost",   edge_cost_boost_);
  corner_cost_boost_ = node_->declare_parameter(name_ + ".corner_cost_boost", corner_cost_boost_);
  border_cost_boost_ = node_->declare_parameter(name_ + ".border_cost_boost", border_cost_boost_);

  inflation_radius_ = node_->declare_parameter(name_ + ".inflation_radius", inflation_radius_);
  inflation_factor_ = node_->declare_parameter(name_ + ".inflation_factor", inflation_factor_);
  inflation_seed_threshold_ = node_->declare_parameter(
    name_ + ".inflation_seed_threshold", inflation_seed_threshold_);

  mesh_msg_.uuid = name_;
  mesh_msg_.header.frame_id = map_frame_;
  costs_msg_.uuid = name_;
  costs_msg_.type = costs_layer_name_;
  costs_msg_.header.frame_id = map_frame_;

  mesh_pub_ = node_->create_publisher<mesh_msgs::msg::MeshGeometryStamped>(
    mesh_topic_, rclcpp::QoS(1).transient_local());
  costs_pub_ = node_->create_publisher<mesh_msgs::msg::MeshVertexCostsStamped>(
    costs_topic_, rclcpp::QoS(1).transient_local());
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/mesh_markers", rclcpp::QoS(1).transient_local());

  octomap_sub_ = node_->create_subscription<octomap_msgs::msg::Octomap>(
    octomap_topic_, rclcpp::QoS(5),
    std::bind(&MeshMappingServer::octomapCallback, this, std::placeholders::_1));

  optimize_timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(optimize_period_),
    std::bind(&MeshMappingServer::optimizeMesh, this));

  RCLCPP_INFO(node_->get_logger(),
    "MeshMappingServer initialized (name=%s, octomap_topic=%s, mesh_topic=%s, "
    "costs_topic=%s, mesh_voxel_size=%.3f, optimize_period=%.1fs)",
    name_.c_str(), octomap_topic_.c_str(), mesh_topic_.c_str(),
    costs_topic_.c_str(), mesh_voxel_size_, optimize_period_);
}

// =============================================================================
// getReadyMesh / getReadyVertexCosts — copy-on-read snapshots
// =============================================================================

std::shared_ptr<mesh_msgs::msg::MeshGeometryStamped> MeshMappingServer::getReadyMesh() const
{
  std::lock_guard<std::mutex> lock(mesh_mutex_);
  return std::make_shared<mesh_msgs::msg::MeshGeometryStamped>(mesh_msg_);
}

std::shared_ptr<mesh_msgs::msg::MeshVertexCostsStamped> MeshMappingServer::getReadyVertexCosts() const
{
  std::lock_guard<std::mutex> lock(mesh_mutex_);
  return std::make_shared<mesh_msgs::msg::MeshVertexCostsStamped>(costs_msg_);
}

// =============================================================================
// mbf_octo_core::MeshMapper interface
// =============================================================================

std::shared_ptr<mbf_octo_core::MeshGraphData> MeshMappingServer::getReadyMeshGraph()
{
  std::lock_guard<std::mutex> lock(mesh_mutex_);
  auto graph = std::make_shared<mbf_octo_core::MeshGraphData>();
  graph->mesh = mesh_msg_;
  graph->vertex_costs = costs_msg_;
  graph->vertex_adjacency = vertex_adjacency_;
  graph->face_adjacency = face_adjacency_;
  graph->face_normals = face_normals_;
  graph->clearance_above = clearance_above_;
  graph->clearance_lateral = clearance_lateral_;
  return graph;
}

std::string MeshMappingServer::getMapFrame() const
{
  std::lock_guard<std::mutex> lock(mesh_mutex_);
  return map_frame_;
}

void MeshMappingServer::publishAdditionalMarkers(
  const visualization_msgs::msg::MarkerArray & ma)
{
  if (marker_pub_) {
    marker_pub_->publish(ma);
  }
}

// =============================================================================
// octomapCallback — cheap incremental path: mesh only the newly-seen bins
// =============================================================================

void MeshMappingServer::octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
  octomap::AbstractOcTree * tree_ptr = msg->binary
    ? octomap_msgs::binaryMsgToMap(*msg)
    : octomap_msgs::fullMsgToMap(*msg);
  if (!tree_ptr) {
    RCLCPP_ERROR(node_->get_logger(), "MeshMappingServer: failed to decode octomap message");
    return;
  }
  std::unique_ptr<octomap::OcTree> octree(dynamic_cast<octomap::OcTree *>(tree_ptr));
  if (!octree) {
    RCLCPP_ERROR(node_->get_logger(), "MeshMappingServer: octomap message is not an OcTree");
    delete tree_ptr;
    return;
  }

  std::lock_guard<std::mutex> lock(mesh_mutex_);
  map_frame_ = msg->header.frame_id;

  // Bin every occupied leaf into the mesh grid. Octomap prunes uniform
  // regions into a single large leaf, so a leaf can be bigger than
  // mesh_voxel_size_ — expand it to cover every bin its cube overlaps so
  // face-culling and greedy merging still see a fully-populated grid.
  std::unordered_set<Bin, BinHash> new_bins;
  for (auto it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {
    if (!octree->isNodeOccupied(*it)) continue;
    octomap::point3d c = it.getCoordinate();
    double half = it.getSize() * 0.5;

    // Drop isolated single-voxel noise before it ever becomes mesh geometry:
    // a stray occupied voxel with no occupied neighbor suppresses the face
    // right below/beside it (face-culling only emits a face when the
    // neighbor bin is unoccupied), punching a hole exactly its own size in
    // otherwise-solid floor/wall. A real surface voxel always has at least
    // one occupied neighbor; a floating speck usually doesn't.
    if (noise_filter_min_neighbors_ > 0) {
      int occupied_neighbors = 0;
      double step = it.getSize();
      static constexpr double offs[6][3] = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
      for (const auto & o : offs) {
        octomap::OcTreeNode * nb = octree->search(
          c.x() + o[0] * step, c.y() + o[1] * step, c.z() + o[2] * step);
        if (nb && octree->isNodeOccupied(nb)) ++occupied_neighbors;
      }
      if (occupied_neighbors < noise_filter_min_neighbors_) continue;
    }

    int32_t bx0 = static_cast<int32_t>(std::floor((c.x() - half) / mesh_voxel_size_));
    int32_t bx1 = static_cast<int32_t>(std::floor((c.x() + half - 1e-9) / mesh_voxel_size_));
    int32_t by0 = static_cast<int32_t>(std::floor((c.y() - half) / mesh_voxel_size_));
    int32_t by1 = static_cast<int32_t>(std::floor((c.y() + half - 1e-9) / mesh_voxel_size_));
    int32_t bz0 = static_cast<int32_t>(std::floor((c.z() - half) / mesh_voxel_size_));
    int32_t bz1 = static_cast<int32_t>(std::floor((c.z() + half - 1e-9) / mesh_voxel_size_));

    for (int32_t bx = bx0; bx <= bx1; ++bx) {
      for (int32_t by = by0; by <= by1; ++by) {
        for (int32_t bz = bz0; bz <= bz1; ++bz) {
          Bin key{bx, by, bz};
          if (occupied_bins_.count(key)) continue;  // already meshed
          new_bins.insert(key);
        }
      }
    }
  }

  if (new_bins.empty()) return;

  for (const auto & b : new_bins) occupied_bins_.insert(b);
  meshBins(new_bins);
  // Keep clearance_above_/clearance_lateral_ in lockstep with the growing
  // vertex count too (see computeClearances()'s incremental-tail comment) —
  // otherwise a planner mid-flight between optimize passes sees a mismatch.
  computeClearances();

  auto stamp = node_->now();
  mesh_msg_.header.stamp = stamp;
  mesh_msg_.header.frame_id = map_frame_;
  mesh_pub_->publish(mesh_msg_);

  // Keep the cost layer's vertex count in lockstep with the geometry on every
  // publish, not just every optimize pass. rviz_mesh_tools_plugins colors a
  // mesh from the most recent MeshVertexCostsStamped it has for that uuid; if
  // the geometry keeps growing between optimize passes while the costs array
  // stays the old (shorter) length, the overlay falls out of sync and stops
  // rendering until the two happen to match again — which is only that one
  // instant right after optimizeMesh() publishes both together. New vertices
  // get a neutral floor_flat_cost_ placeholder until the next optimize pass
  // classifies them for real.
  size_t old_size = costs_msg_.mesh_vertex_costs.costs.size();
  size_t new_size = mesh_msg_.mesh_geometry.vertices.size();
  if (new_size > old_size) {
    costs_msg_.mesh_vertex_costs.costs.resize(new_size, static_cast<float>(floor_flat_cost_));
  }
  costs_msg_.uuid = mesh_msg_.uuid;
  costs_msg_.type = costs_layer_name_;
  costs_msg_.header.stamp = stamp;
  costs_msg_.header.frame_id = map_frame_;
  costs_pub_->publish(costs_msg_);
}

// =============================================================================
// optimizeMesh — periodic full remesh (merges across old seams) + cost layer
// =============================================================================

void MeshMappingServer::optimizeMesh()
{
  std::lock_guard<std::mutex> lock(mesh_mutex_);
  if (occupied_bins_.empty()) return;

  size_t new_bins_since_last = occupied_bins_.size() - bins_at_last_optimize_;
  if (new_bins_since_last < static_cast<size_t>(std::max(0, min_new_bins_for_optimize_))) {
    RCLCPP_DEBUG(node_->get_logger(),
      "MeshMappingServer: skipping optimize pass (%zu new bins < %d threshold)",
      new_bins_since_last, min_new_bins_for_optimize_);
    return;
  }

  mesh_msg_.mesh_geometry.vertices.clear();
  mesh_msg_.mesh_geometry.vertex_normals.clear();
  mesh_msg_.mesh_geometry.faces.clear();
  vertex_index_.clear();
  vertex_corners_.clear();
  vertex_features_.clear();
  floor_top_height_.clear();
  smoothed_floor_height_.clear();
  clearance_above_.clear();
  clearance_lateral_.clear();

  // Re-meshing the full occupancy grid at once (instead of only the bins from
  // one incremental update) is what lets greedy merging combine faces across
  // old batch boundaries into bigger, smoother rectangles.
  meshBins(occupied_bins_);
  // Guarantee <= 2 triangles per edge before anything downstream (starting
  // with buildGraphTopology()) assumes that.
  enforceManifoldTriangles();
  buildGraphTopology();
  // Smoothing needs Gv (just built above) to know each vertex's
  // floor-neighbors; rebuild topology afterward so face_normals_ (derived
  // from vertex positions) reflect the smoothed geometry, not the original
  // stepped one.
  smoothRampTransitions();
  buildGraphTopology();
  computeClearances();
  buildSmoothedFloorHeights();
  computeVertexCosts();
  inflateCostsFromHazards();

  auto stamp = node_->now();
  mesh_msg_.header.stamp = stamp;
  mesh_msg_.header.frame_id = map_frame_;
  mesh_pub_->publish(mesh_msg_);

  costs_msg_.header.stamp = stamp;
  costs_msg_.header.frame_id = map_frame_;
  costs_pub_->publish(costs_msg_);

  bins_at_last_optimize_ = occupied_bins_.size();

  RCLCPP_INFO(node_->get_logger(),
    "MeshMappingServer: optimize pass — %zu bins -> %zu vertices, %zu faces",
    occupied_bins_.size(), mesh_msg_.mesh_geometry.vertices.size(),
    mesh_msg_.mesh_geometry.faces.size());
}

// =============================================================================
// isOccupied / vertexIndexFor
// =============================================================================

bool MeshMappingServer::isOccupied(const Bin & bin) const
{
  return occupied_bins_.count(bin) > 0;
}

uint32_t MeshMappingServer::vertexIndexFor(const Bin & corner)
{
  auto it = vertex_index_.find(corner);
  if (it != vertex_index_.end()) return it->second;

  geometry_msgs::msg::Point p;
  p.x = corner[0] * mesh_voxel_size_;
  p.y = corner[1] * mesh_voxel_size_;
  p.z = corner[2] * mesh_voxel_size_;

  uint32_t idx = static_cast<uint32_t>(mesh_msg_.mesh_geometry.vertices.size());
  mesh_msg_.mesh_geometry.vertices.push_back(p);
  // Placeholder; overwritten with the real face normal by the caller right
  // after this returns. Vertices shared by faces of different orientation
  // (e.g. a wall/floor corner) end up with whichever face touched them last —
  // acceptable for a low-poly nav mesh, not meant for smooth shading.
  mesh_msg_.mesh_geometry.vertex_normals.push_back(geometry_msgs::msg::Point());
  vertex_corners_.push_back(corner);
  vertex_features_.push_back(VertexFeatures{});

  vertex_index_.emplace(corner, idx);
  return idx;
}

// =============================================================================
// meshBins — axis-aligned greedy meshing, restricted to `bins`
// =============================================================================

void MeshMappingServer::meshBins(const std::unordered_set<Bin, BinHash> & bins)
{
  if (bins.empty()) return;

  Bin bmin{std::numeric_limits<int32_t>::max(), std::numeric_limits<int32_t>::max(),
           std::numeric_limits<int32_t>::max()};
  Bin bmax{std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::min(),
           std::numeric_limits<int32_t>::min()};
  for (const auto & b : bins) {
    for (int a = 0; a < 3; ++a) {
      bmin[a] = std::min(bmin[a], b[a]);
      bmax[a] = std::max(bmax[a], b[a]);
    }
  }

  double stair_riser_max_bins = stair_riser_max_height_ / mesh_voxel_size_;
  // Cap how far the greedy merge grows a single quad, so one huge flat area
  // doesn't collapse into one giant rectangle — keeps every triangle edge
  // (other than the merged quad's own diagonal) at or below max_triangle_edge_.
  int32_t max_dim_bins = std::max(1, static_cast<int32_t>(std::round(max_triangle_edge_ / mesh_voxel_size_)));

  // axis = the face normal direction; u, v = the other two axes (cyclic,
  // right-handed: x->y->z), spanning the face plane.
  for (int axis = 0; axis < 3; ++axis) {
    int u = (axis + 1) % 3;
    int v = (axis + 2) % 3;
    int32_t su = bmax[u] - bmin[u] + 1;
    int32_t sv = bmax[v] - bmin[v] + 1;
    if (su <= 0 || sv <= 0) continue;

    for (int dir = -1; dir <= 1; dir += 2) {
      for (int32_t i = bmin[axis]; i <= bmax[axis]; ++i) {
        std::vector<uint8_t> mask(static_cast<size_t>(su) * static_cast<size_t>(sv), 0);
        auto maskIdx = [&](int32_t uu, int32_t vv) {
          return static_cast<size_t>(uu - bmin[u]) * static_cast<size_t>(sv) + (vv - bmin[v]);
        };

        bool any = false;
        for (int32_t uu = bmin[u]; uu <= bmax[u]; ++uu) {
          for (int32_t vv = bmin[v]; vv <= bmax[v]; ++vv) {
            Bin bin{};
            bin[axis] = i; bin[u] = uu; bin[v] = vv;
            if (!bins.count(bin)) continue;
            Bin nb = bin;
            nb[axis] += dir;
            if (isOccupied(nb)) continue;  // interior face — skip
            mask[maskIdx(uu, vv)] = 1;
            any = true;
          }
        }
        if (!any) continue;

        // Greedy rectangle merge over the 2D mask (standard voxel-mesher
        // sweep: grow each unmerged cell as wide as possible, then as tall
        // as possible while every cell in that width stays set).
        std::vector<uint8_t> merged(mask.size(), 0);
        for (int32_t vv = bmin[v]; vv <= bmax[v]; ++vv) {
          for (int32_t uu = bmin[u]; uu <= bmax[u]; ++uu) {
            size_t idx0 = maskIdx(uu, vv);
            if (!mask[idx0] || merged[idx0]) continue;

            int32_t w = 1;
            while (w < max_dim_bins && uu + w <= bmax[u]) {
              size_t idx = maskIdx(uu + w, vv);
              if (!mask[idx] || merged[idx]) break;
              ++w;
            }

            int32_t h = 1;
            bool grow = true;
            while (h < max_dim_bins && vv + h <= bmax[v] && grow) {
              for (int32_t ww = 0; ww < w; ++ww) {
                size_t idx = maskIdx(uu + ww, vv + h);
                if (!mask[idx] || merged[idx]) { grow = false; break; }
              }
              if (grow) ++h;
            }

            for (int32_t ww = 0; ww < w; ++ww) {
              for (int32_t hh = 0; hh < h; ++hh) {
                merged[maskIdx(uu + ww, vv + hh)] = 1;
              }
            }

            // Emit the merged quad. The face plane sits on the lattice
            // boundary of the slice (i, or i+1 for the positive-direction
            // face), spanning [uu, uu+w) x [vv, vv+h) in bin units.
            int32_t face_i = i + (dir > 0 ? 1 : 0);
            Bin c00{}, c10{}, c11{}, c01{};
            c00[axis] = c10[axis] = c11[axis] = c01[axis] = face_i;
            c00[u] = uu;     c00[v] = vv;
            c10[u] = uu + w; c10[v] = vv;
            c11[u] = uu + w; c11[v] = vv + h;
            c01[u] = uu;     c01[v] = vv + h;

            uint32_t i00 = vertexIndexFor(c00);
            uint32_t i10 = vertexIndexFor(c10);
            uint32_t i11 = vertexIndexFor(c11);
            uint32_t i01 = vertexIndexFor(c01);

            geometry_msgs::msg::Point normal;
            normal.x = (axis == 0) ? static_cast<double>(dir) : 0.0;
            normal.y = (axis == 1) ? static_cast<double>(dir) : 0.0;
            normal.z = (axis == 2) ? static_cast<double>(dir) : 0.0;
            mesh_msg_.mesh_geometry.vertex_normals[i00] = normal;
            mesh_msg_.mesh_geometry.vertex_normals[i10] = normal;
            mesh_msg_.mesh_geometry.vertex_normals[i11] = normal;
            mesh_msg_.mesh_geometry.vertex_normals[i01] = normal;

            // --- per-vertex feature bookkeeping, consumed by computeVertexCosts() ---
            int dir_idx = axis * 2 + (dir > 0 ? 0 : 1);  // 0=+x,1=-x,2=+y,3=-y,4=+z,5=-z
            bool is_wall_normal_face = (axis != 2);
            // axis 0 (x): u=y, v=z -> h measures the face's z-extent.
            // axis 1 (y): u=z, v=x -> w measures the face's z-extent.
            bool tall = false;
            if (is_wall_normal_face) {
              int32_t vertical_extent_bins = (axis == 0) ? h : w;
              tall = static_cast<double>(vertical_extent_bins) > stair_riser_max_bins;
            }
            for (uint32_t vidx : {i00, i10, i11, i01}) {
              vertex_features_[vidx].dirs[dir_idx] = true;
              if (is_wall_normal_face) {
                if (tall) vertex_features_[vidx].any_tall_vertical_face = true;
                else      vertex_features_[vidx].any_short_vertical_face = true;
              }
            }
            if (axis == 2 && dir > 0) {
              // Top (+Z) floor face: record height for floorSlopeCost() lookups.
              floor_top_height_[{c00[0], c00[1]}] = face_i;
              floor_top_height_[{c10[0], c10[1]}] = face_i;
              floor_top_height_[{c11[0], c11[1]}] = face_i;
              floor_top_height_[{c01[0], c01[1]}] = face_i;
            }

            // Winding: positive-direction faces go u-ascending/v-ascending;
            // negative-direction faces reverse it, so both wind CCW as seen
            // from the direction the normal points.
            mesh_msgs::msg::MeshTriangleIndices t1, t2;
            if (dir > 0) {
              t1.vertex_indices = {i00, i10, i11};
              t2.vertex_indices = {i00, i11, i01};
            } else {
              t1.vertex_indices = {i00, i11, i10};
              t2.vertex_indices = {i00, i01, i11};
            }
            mesh_msg_.mesh_geometry.faces.push_back(t1);
            mesh_msg_.mesh_geometry.faces.push_back(t2);
          }
        }
      }
    }
  }
}

// =============================================================================
// buildSmoothedFloorHeights
// =============================================================================

void MeshMappingServer::buildSmoothedFloorHeights()
{
  smoothed_floor_height_.clear();
  if (floor_top_height_.empty()) return;

  double cell_m = std::max(slope_smoothing_radius_, mesh_voxel_size_);

  struct Accum { double sum = 0.0; int count = 0; };
  std::unordered_map<std::pair<int32_t, int32_t>, Accum, XYHash> pooled;
  for (const auto & [xy, bz] : floor_top_height_) {
    double wx = xy.first  * mesh_voxel_size_;
    double wy = xy.second * mesh_voxel_size_;
    double wz = bz * mesh_voxel_size_;
    std::pair<int32_t, int32_t> cell{
      static_cast<int32_t>(std::floor(wx / cell_m)),
      static_cast<int32_t>(std::floor(wy / cell_m))};
    Accum & acc = pooled[cell];
    acc.sum += wz;
    acc.count += 1;
  }

  smoothed_floor_height_.reserve(pooled.size());
  for (const auto & [cell, acc] : pooled) {
    smoothed_floor_height_[cell] = acc.sum / acc.count;
  }
}

// =============================================================================
// floorSlopeCost
// =============================================================================

float MeshMappingServer::floorSlopeCost(const Bin & corner) const
{
  double cell_m = std::max(slope_smoothing_radius_, mesh_voxel_size_);
  int32_t cx = static_cast<int32_t>(std::floor((corner[0] * mesh_voxel_size_) / cell_m));
  int32_t cy = static_cast<int32_t>(std::floor((corner[1] * mesh_voxel_size_) / cell_m));

  auto self_it = smoothed_floor_height_.find({cx, cy});
  double self_h = (self_it != smoothed_floor_height_.end())
    ? self_it->second : corner[2] * mesh_voxel_size_;

  // Compare pooled (averaged) heights of neighboring cells rather than raw
  // per-bin heights: this is what cancels out sub-cell surface texture (e.g.
  // a ramp built from evenly-spaced wooden slats reads as a staircase at bin
  // granularity) while still tracking a genuine, sustained incline.
  double max_dh = 0.0;
  static constexpr int32_t offs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  for (const auto & o : offs) {
    auto it = smoothed_floor_height_.find({cx + o[0], cy + o[1]});
    if (it != smoothed_floor_height_.end()) {
      max_dh = std::max(max_dh, std::abs(it->second - self_h));
    }
  }

  double angle = std::atan2(max_dh, cell_m);
  double t = std::min(1.0, angle / (M_PI * 0.5));
  return static_cast<float>(floor_flat_cost_ + t * (wall_cost_ - floor_flat_cost_));
}

// =============================================================================
// classifyRealStaircases
// =============================================================================

std::vector<bool> MeshMappingServer::classifyRealStaircases() const
{
  const size_t n = mesh_msg_.mesh_geometry.vertices.size();
  std::vector<bool> is_real_stair(n, false);

  std::vector<uint32_t> candidates;
  for (size_t idx = 0; idx < vertex_features_.size(); ++idx) {
    const VertexFeatures & f = vertex_features_[idx];
    if (f.any_short_vertical_face && !f.any_tall_vertical_face) {
      candidates.push_back(static_cast<uint32_t>(idx));
    }
  }
  if (candidates.size() < static_cast<size_t>(std::max(1, stair_min_chain_length_))) {
    return is_real_stair;
  }

  // Union-find over candidates, connecting any two within stair_max_xy_dist_
  // of each other. O(candidates^2), same tradeoff OctoMappingServer's own
  // adjacency build accepts — fine for the handful of short-riser vertices a
  // stair/ramp region actually produces.
  std::vector<uint32_t> parent(candidates.size());
  for (size_t i = 0; i < parent.size(); ++i) parent[i] = static_cast<uint32_t>(i);
  auto find = [&](uint32_t x) {
    while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
    return x;
  };
  auto unite = [&](uint32_t a, uint32_t b) {
    a = find(a); b = find(b);
    if (a != b) parent[a] = b;
  };

  double max_xy_sq = stair_max_xy_dist_ * stair_max_xy_dist_;
  for (size_t i = 0; i < candidates.size(); ++i) {
    const Bin & ci = vertex_corners_[candidates[i]];
    for (size_t j = i + 1; j < candidates.size(); ++j) {
      const Bin & cj = vertex_corners_[candidates[j]];
      double dx = (ci[0] - cj[0]) * mesh_voxel_size_;
      double dy = (ci[1] - cj[1]) * mesh_voxel_size_;
      if (dx * dx + dy * dy <= max_xy_sq) {
        unite(static_cast<uint32_t>(i), static_cast<uint32_t>(j));
      }
    }
  }

  std::unordered_map<uint32_t, int> group_size;
  for (size_t i = 0; i < candidates.size(); ++i) ++group_size[find(static_cast<uint32_t>(i))];

  for (size_t i = 0; i < candidates.size(); ++i) {
    if (group_size[find(static_cast<uint32_t>(i))] >= stair_min_chain_length_) {
      is_real_stair[candidates[i]] = true;
    }
  }
  return is_real_stair;
}

// =============================================================================
// computeFrontierVertices
// =============================================================================

std::vector<bool> MeshMappingServer::computeFrontierVertices() const
{
  const size_t n = vertex_corners_.size();
  std::vector<bool> is_frontier(n, false);

  int32_t radius_bins = std::max(
    1, static_cast<int32_t>(std::round(frontier_check_radius_ / mesh_voxel_size_)));

  static constexpr int32_t offs[8][2] = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

  for (size_t i = 0; i < n; ++i) {
    if (i >= vertex_features_.size() || !vertex_features_[i].dirs[4]) continue;  // floor-top only
    const Bin & c = vertex_corners_[i];
    // "Occupied floor nearby in direction o" = an occupied bin at
    // (c.x + o.x*r, c.y + o.y*r, c.z - 1) — one bin below this vertex's own
    // free-air layer, i.e. the actual floor material — for some r up to
    // radius_bins. Genuinely nothing there in even one of the 8 directions
    // means this vertex sits at the true edge of explored space.
    for (const auto & o : offs) {
      bool found = false;
      for (int32_t r = 1; r <= radius_bins; ++r) {
        Bin probe{c[0] + o[0] * r, c[1] + o[1] * r, c[2] - 1};
        if (occupied_bins_.count(probe)) { found = true; break; }
      }
      if (!found) { is_frontier[i] = true; break; }
    }
  }
  return is_frontier;
}

// =============================================================================
// computeVertexCosts
// =============================================================================

void MeshMappingServer::computeVertexCosts()
{
  const size_t n = mesh_msg_.mesh_geometry.vertices.size();
  costs_msg_.uuid = mesh_msg_.uuid;
  costs_msg_.type = costs_layer_name_;
  costs_msg_.mesh_vertex_costs.costs.assign(n, 0.0f);

  std::vector<bool> is_real_stair = classifyRealStaircases();
  std::vector<bool> is_frontier = computeFrontierVertices();

  for (size_t idx = 0; idx < n; ++idx) {
    const VertexFeatures & f = vertex_features_[idx];
    bool touches_x    = f.dirs[0] || f.dirs[1];
    bool touches_y    = f.dirs[2] || f.dirs[3];
    bool touches_up   = f.dirs[4];
    bool touches_down = f.dirs[5];
    bool touches_vert = touches_up || touches_down;
    int num_categories = (touches_x ? 1 : 0) + (touches_y ? 1 : 0) + (touches_vert ? 1 : 0);

    float cost = 0.0f;
    if (num_categories > 0) {
      bool short_riser = (touches_x || touches_y) && !f.any_tall_vertical_face;
      bool is_validated_stair = short_riser && is_real_stair[idx];
      if (is_validated_stair) {
        // Explicitly cheap: part of a genuine multi-step staircase run —
        // normal, walkable terrain, not a hazard like a real wall edge.
        cost = static_cast<float>(stair_cost_);
      } else {
        // Either a genuine wall/floor/ceiling, or a short bump that looks
        // stair-like but ISN'T part of a validated staircase run (e.g. a
        // curb, a rock) — treated as a real obstacle, at wall-level cost.
        float wall_component  = (touches_x || touches_y) ? static_cast<float>(wall_cost_) : 0.0f;
        float floor_component = touches_up   ? floorSlopeCost(vertex_corners_[idx])       : 0.0f;
        float ceil_component  = touches_down ? static_cast<float>(wall_cost_)             : 0.0f;
        cost = std::max({wall_component, floor_component, ceil_component});
        if (num_categories >= 2) cost = std::min(1.0f, cost + static_cast<float>(edge_cost_boost_));
        if (num_categories >= 3) cost = std::min(1.0f, cost + static_cast<float>(corner_cost_boost_));
      }
    }

    if (is_frontier[idx]) {
      cost = std::min(1.0f, cost + static_cast<float>(border_cost_boost_));
    }
    costs_msg_.mesh_vertex_costs.costs[idx] = cost;
  }
}

// =============================================================================
// inflateCostsFromHazards
// =============================================================================

void MeshMappingServer::inflateCostsFromHazards()
{
  auto & costs = costs_msg_.mesh_vertex_costs.costs;
  const size_t n = costs.size();
  if (n == 0 || inflation_radius_ <= 0.0) return;

  // Reuses the persistent Gv built by buildGraphTopology() (must run first)
  // instead of rebuilding its own adjacency list every pass.
  struct SpreadEntry { double dist; uint32_t idx; float source_cost; };
  auto cmp = [](const SpreadEntry & a, const SpreadEntry & b) { return a.dist > b.dist; };
  std::priority_queue<SpreadEntry, std::vector<SpreadEntry>, decltype(cmp)> pq(cmp);
  std::vector<double> best_dist(n, std::numeric_limits<double>::max());
  std::vector<float> spread(n, 0.0f);

  for (size_t idx = 0; idx < n; ++idx) {
    if (costs[idx] >= static_cast<float>(inflation_seed_threshold_)) {
      spread[idx] = costs[idx];
      best_dist[idx] = 0.0;
      pq.push({0.0, static_cast<uint32_t>(idx), costs[idx]});
    }
  }

  while (!pq.empty()) {
    SpreadEntry top = pq.top();
    pq.pop();
    if (top.dist > best_dist[top.idx] + 1e-9) continue;
    for (const auto & edge : vertex_adjacency_[top.idx]) {
      double new_dist = top.dist + edge.length;
      if (new_dist >= inflation_radius_) continue;
      float new_val = static_cast<float>(
        top.source_cost * (1.0 - new_dist / inflation_radius_) * inflation_factor_);
      if (new_val > spread[edge.to] + 1e-6f) {
        spread[edge.to] = new_val;
        best_dist[edge.to] = new_dist;
        pq.push({new_dist, edge.to, top.source_cost});
      }
    }
  }

  for (size_t idx = 0; idx < n; ++idx) {
    costs[idx] = std::max(costs[idx], spread[idx]);
  }
}

// =============================================================================
// buildGraphTopology — Gv (vertex_adjacency_) + Gt (face_adjacency_, face_normals_)
// =============================================================================

void MeshMappingServer::buildGraphTopology()
{
  const auto & verts = mesh_msg_.mesh_geometry.vertices;
  const auto & faces = mesh_msg_.mesh_geometry.faces;
  const size_t nv = verts.size();
  const size_t nf = faces.size();

  // ---- Gv: vertex adjacency, edge length cached in metres -------------------
  vertex_adjacency_.assign(nv, {});
  for (const auto & tri : faces) {
    uint32_t idxs[3] = {tri.vertex_indices[0], tri.vertex_indices[1], tri.vertex_indices[2]};
    for (int e = 0; e < 3; ++e) {
      uint32_t a = idxs[e], b = idxs[(e + 1) % 3];
      double dx = verts[a].x - verts[b].x;
      double dy = verts[a].y - verts[b].y;
      double dz = verts[a].z - verts[b].z;
      float len = static_cast<float>(std::sqrt(dx * dx + dy * dy + dz * dz));
      vertex_adjacency_[a].push_back({b, len});
      vertex_adjacency_[b].push_back({a, len});
    }
  }

  // ---- Gv patch: bridge T-junctions between floor-top vertices ------------
  // A big greedy-merged quad's edge isn't split to match a smaller
  // neighboring quad's corner, so that corner has no shared-triangle-edge
  // connection into the middle of the big quad's edge even though the mesh
  // visually touches there. Left unpatched, every such seam reads as a fake
  // disconnection to Dijkstra. Bucket floor-top vertices by (bx,by) and
  // connect any pair in bin-adjacent columns whose height differs by no more
  // than lattice_bridge_max_dz_ (skips real steps/walls, not just T-junctions).
  {
    std::unordered_map<std::pair<int32_t, int32_t>, std::vector<uint32_t>, XYHash> floor_by_xy;
    for (size_t i = 0; i < nv; ++i) {
      if (i < vertex_features_.size() && vertex_features_[i].dirs[4]) {
        floor_by_xy[{vertex_corners_[i][0], vertex_corners_[i][1]}].push_back(
          static_cast<uint32_t>(i));
      }
    }

    static constexpr int32_t offs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    for (const auto & [xy, idxs_here] : floor_by_xy) {
      for (const auto & o : offs) {
        auto it = floor_by_xy.find({xy.first + o[0], xy.second + o[1]});
        if (it == floor_by_xy.end()) continue;
        for (uint32_t a : idxs_here) {
          for (uint32_t b : it->second) {
            double dz = std::abs(verts[a].z - verts[b].z);
            if (dz > lattice_bridge_max_dz_) continue;
            bool already = false;
            for (const auto & e : vertex_adjacency_[a]) {
              if (e.to == b) { already = true; break; }
            }
            if (already) continue;
            double dx = verts[a].x - verts[b].x;
            double dy = verts[a].y - verts[b].y;
            float len = static_cast<float>(std::sqrt(dx * dx + dy * dy + dz * dz));
            vertex_adjacency_[a].push_back({b, len});
            vertex_adjacency_[b].push_back({a, len});
          }
        }
      }
    }
  }

  // ---- Gt: face adjacency (up to 3 neighbors, one per edge) + face normals --
  face_normals_.assign(nf, {0.0f, 0.0f, 1.0f});
  face_adjacency_.assign(nf, {-1, -1, -1});

  auto edgeKey = [](uint32_t a, uint32_t b) -> uint64_t {
    uint32_t lo = std::min(a, b), hi = std::max(a, b);
    return (static_cast<uint64_t>(lo) << 32) | hi;
  };
  // Undirected edge -> (owning face index, local edge index 0..2) of the
  // first triangle seen using it; resolved into face_adjacency_ once the
  // second triangle sharing that edge is found. An edge with only one owner
  // by the end is a mesh boundary edge (stays -1).
  std::unordered_map<uint64_t, std::pair<int32_t, int32_t>> edge_owner;
  edge_owner.reserve(nf * 3);

  for (size_t f = 0; f < nf; ++f) {
    const auto & tri = faces[f];
    uint32_t idxs[3] = {tri.vertex_indices[0], tri.vertex_indices[1], tri.vertex_indices[2]};

    const auto & p0 = verts[idxs[0]];
    const auto & p1 = verts[idxs[1]];
    const auto & p2 = verts[idxs[2]];
    double e1x = p1.x - p0.x, e1y = p1.y - p0.y, e1z = p1.z - p0.z;
    double e2x = p2.x - p0.x, e2y = p2.y - p0.y, e2z = p2.z - p0.z;
    double nx = e1y * e2z - e1z * e2y;
    double ny = e1z * e2x - e1x * e2z;
    double nz = e1x * e2y - e1y * e2x;
    double norm = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (norm > 1e-12) {
      face_normals_[f] = {static_cast<float>(nx / norm),
                           static_cast<float>(ny / norm),
                           static_cast<float>(nz / norm)};
    }

    for (int e = 0; e < 3; ++e) {
      uint32_t a = idxs[e], b = idxs[(e + 1) % 3];
      uint64_t key = edgeKey(a, b);
      auto it = edge_owner.find(key);
      if (it == edge_owner.end()) {
        edge_owner.emplace(key, std::make_pair(static_cast<int32_t>(f), e));
      } else {
        int32_t other_f = it->second.first;
        int32_t other_e = it->second.second;
        face_adjacency_[f][e] = other_f;
        face_adjacency_[other_f][other_e] = static_cast<int32_t>(f);
      }
    }
  }
}

// =============================================================================
// computeClearances / computeClearanceAbove / computeClearanceLateral
// =============================================================================

float MeshMappingServer::computeClearanceAbove(const Bin & corner) const
{
  int32_t max_steps = static_cast<int32_t>(
    std::ceil(clearance_scan_max_height_ / mesh_voxel_size_));
  // The band from the floor up to collision_check_height_offset_ is never
  // occupancy-checked at all — it's assumed clear (leg/foot space, or just
  // ground-level texture noise). clear_steps therefore starts pre-credited
  // with that band; scanning for real obstructions only begins above it,
  // which is where the robot's actual body needs room.
  int32_t offset_bins = std::max(
    0, static_cast<int32_t>(std::round(collision_check_height_offset_ / mesh_voxel_size_)));
  int32_t clear_steps = offset_bins;
  Bin probe = corner;
  for (int32_t s = offset_bins + 1; s <= max_steps; ++s) {
    probe[2] = corner[2] + s;
    if (occupied_bins_.count(probe)) break;
    clear_steps = s;
  }
  return static_cast<float>(
    std::min(static_cast<double>(clear_steps) * mesh_voxel_size_, clearance_scan_max_height_));
}

float MeshMappingServer::computeClearanceLateral(const Bin & corner) const
{
  int32_t max_steps = static_cast<int32_t>(
    std::ceil(clearance_scan_max_radius_ / mesh_voxel_size_));
  // Scan at roughly mid-body height above the vertex, not at its own exact
  // height — see collision_check_height_offset_'s doc comment for why (floor
  // height noise vs. genuine walls).
  int32_t z_offset_bins = std::max(
    1, static_cast<int32_t>(std::round(collision_check_height_offset_ / mesh_voxel_size_)));
  int32_t scan_z = corner[2] + z_offset_bins;

  double min_clear = clearance_scan_max_radius_;
  for (int d = 0; d < clearance_scan_num_dirs_; ++d) {
    double angle = d * (2.0 * M_PI / clearance_scan_num_dirs_);
    double dxf = std::cos(angle), dyf = std::sin(angle);
    for (int32_t s = 1; s <= max_steps; ++s) {
      Bin probe{
        corner[0] + static_cast<int32_t>(std::round(dxf * s)),
        corner[1] + static_cast<int32_t>(std::round(dyf * s)),
        scan_z};
      if (occupied_bins_.count(probe)) {
        double dist = (s - 1) * mesh_voxel_size_;  // clearance just before the obstacle
        min_clear = std::min(min_clear, dist);
        break;
      }
    }
  }
  return static_cast<float>(std::max(0.0, min_clear));
}

void MeshMappingServer::computeClearances()
{
  // Only fills in the newly-appended tail (clearance_above_.size() ..
  // vertex_corners_.size()), so this is cheap to call from the incremental
  // octomapCallback() path too, not just the full optimizeMesh() rebuild.
  // optimizeMesh() clears both arrays first, so there "old size" is 0 there
  // and this naturally becomes a full recompute — required, since a full
  // remesh reshuffles every vertex index.
  const size_t n = vertex_corners_.size();
  const size_t old_n = clearance_above_.size();
  clearance_above_.resize(n);
  clearance_lateral_.resize(n);
  for (size_t i = old_n; i < n; ++i) {
    clearance_above_[i] = computeClearanceAbove(vertex_corners_[i]);
    clearance_lateral_[i] = computeClearanceLateral(vertex_corners_[i]);
  }
}

// =============================================================================
// smoothRampTransitions
// =============================================================================

void MeshMappingServer::smoothRampTransitions()
{
  auto & verts = mesh_msg_.mesh_geometry.vertices;
  const size_t n = verts.size();
  if (n == 0 || ramp_smoothing_iterations_ <= 0) return;

  // Eligible = floor-top vertices that also touch a short (stair-riser-scale)
  // vertical face — exactly the tread/riser edges a voxelized ramp shows up
  // as. Vertices on a genuine tall wall are never eligible, regardless of
  // height difference, so real walls/steps stay crisp.
  std::vector<bool> eligible(n, false);
  for (size_t i = 0; i < n; ++i) {
    const VertexFeatures & f = vertex_features_[i];
    if (f.dirs[4] && f.any_short_vertical_face && !f.any_tall_vertical_face) {
      eligible[i] = true;
    }
  }

  std::vector<double> z(n);
  for (size_t i = 0; i < n; ++i) z[i] = verts[i].z;

  for (int iter = 0; iter < ramp_smoothing_iterations_; ++iter) {
    std::vector<double> new_z = z;
    for (size_t i = 0; i < n; ++i) {
      if (!eligible[i]) continue;
      double sum = z[i];
      int count = 1;
      for (const auto & edge : vertex_adjacency_[i]) {
        uint32_t nb = edge.to;
        if (!vertex_features_[nb].dirs[4]) continue;  // neighbor must be a floor vertex too
        if (std::abs(z[nb] - z[i]) > max_ramp_rise_) continue;  // real step/wall — don't blend across it
        sum += z[nb];
        ++count;
      }
      if (count > 1) new_z[i] = sum / count;
    }
    z.swap(new_z);
  }

  for (size_t i = 0; i < n; ++i) {
    if (eligible[i]) verts[i].z = z[i];
  }
}

// =============================================================================
// enforceManifoldTriangles
// =============================================================================

void MeshMappingServer::enforceManifoldTriangles()
{
  auto & faces = mesh_msg_.mesh_geometry.faces;
  if (faces.empty()) return;

  auto edgeKey = [](uint32_t a, uint32_t b) -> uint64_t {
    uint32_t lo = std::min(a, b), hi = std::max(a, b);
    return (static_cast<uint64_t>(lo) << 32) | hi;
  };

  // Total triangle count per undirected edge, over the whole mesh.
  std::unordered_map<uint64_t, int> edge_count;
  edge_count.reserve(faces.size() * 3);
  for (const auto & tri : faces) {
    edge_count[edgeKey(tri.vertex_indices[0], tri.vertex_indices[1])]++;
    edge_count[edgeKey(tri.vertex_indices[1], tri.vertex_indices[2])]++;
    edge_count[edgeKey(tri.vertex_indices[2], tri.vertex_indices[0])]++;
  }
  size_t non_manifold_edges = 0;
  for (const auto & [k, cnt] : edge_count) {
    if (cnt >= 3) ++non_manifold_edges;
  }
  if (non_manifold_edges == 0) return;  // already fully manifold — common case, skip the rewrite

  // Manifold = exactly 1 (boundary — normal, e.g. the edge of what's mapped
  // so far, or a T-junction gap — left alone) or 2 (interior) triangles per
  // edge. For any edge with 3+, keep the first 2 triangles that claim it (in
  // face order) and drop every triangle after that. A dropped triangle just
  // leaves a small boundary gap where it was; it never breaks a *different*
  // edge's manifold-ness, since we only ever remove, never add or move,
  // triangles.
  std::unordered_map<uint64_t, int> edge_kept;
  std::vector<mesh_msgs::msg::MeshTriangleIndices> kept_faces;
  kept_faces.reserve(faces.size());
  size_t dropped = 0;

  for (const auto & tri : faces) {
    uint64_t keys[3] = {
      edgeKey(tri.vertex_indices[0], tri.vertex_indices[1]),
      edgeKey(tri.vertex_indices[1], tri.vertex_indices[2]),
      edgeKey(tri.vertex_indices[2], tri.vertex_indices[0])};

    bool keep = true;
    for (uint64_t k : keys) {
      if (edge_count[k] >= 3 && edge_kept[k] >= 2) { keep = false; break; }
    }
    if (keep) {
      for (uint64_t k : keys) ++edge_kept[k];
      kept_faces.push_back(tri);
    } else {
      ++dropped;
    }
  }

  RCLCPP_WARN(node_->get_logger(),
    "MeshMappingServer: found %zu non-manifold edge(s) (used by 3+ triangles); "
    "dropped %zu triangle(s) to restore manifold topology (%zu -> %zu faces)",
    non_manifold_edges, dropped, faces.size(), kept_faces.size());
  faces = std::move(kept_faces);
}

}  // namespace mbf_octo_nav
