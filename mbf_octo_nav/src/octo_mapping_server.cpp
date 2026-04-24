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

#include "mbf_octo_nav/octo_mapping_server.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <queue>
#include <sstream>
#include <thread>
#include <vector>

#include <octomap_msgs/conversions.h>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/point.hpp>

using namespace mbf_octo_core;

namespace mbf_octo_nav
{

// =============================================================================
// Constructor / Destructor
// =============================================================================

OctoMappingServer::OctoMappingServer()
{
  min_bound_ = {0.0, 0.0, 0.0};
  max_bound_ = {0.0, 0.0, 0.0};
}

OctoMappingServer::~OctoMappingServer()
{
  saveMapsOnShutdown();
}

// =============================================================================
// initialize
// =============================================================================

void OctoMappingServer::initialize(const std::string & name,
                                    const rclcpp::Node::SharedPtr & node)
{
  name_ = name;
  node_ = node;

  // --- Octomap subscription -------------------------------------------------
  octomap_topic_ = node_->declare_parameter(name_ + ".octomap_topic", octomap_topic_);
  octomap_sub_ = node_->create_subscription<octomap_msgs::msg::Octomap>(
    octomap_topic_, 1,
    std::bind(&OctoMappingServer::octomapCallback, this, std::placeholders::_1));

  enable_octomap_updates_ = node_->declare_parameter(
    name_ + ".enable_octomap_updates", enable_octomap_updates_);
  incremental_graph_build_ = node_->declare_parameter(
    name_ + ".incremental_graph_build", incremental_graph_build_);

  // --- Sensor model ---------------------------------------------------------
  octomap_prob_hit_  = node_->declare_parameter(name_ + ".octomap_prob_hit",  octomap_prob_hit_);
  octomap_prob_miss_ = node_->declare_parameter(name_ + ".octomap_prob_miss", octomap_prob_miss_);
  octomap_thres_     = node_->declare_parameter(name_ + ".octomap_thres",     octomap_thres_);
  octomap_clamp_min_ = node_->declare_parameter(name_ + ".octomap_clamp_min", octomap_clamp_min_);
  octomap_clamp_max_ = node_->declare_parameter(name_ + ".octomap_clamp_max", octomap_clamp_max_);

  // --- Graph build parameters -----------------------------------------------
  voxel_size_            = node_->declare_parameter(name_ + ".voxel_size",            voxel_size_);
  z_threshold_           = node_->declare_parameter(name_ + ".z_threshold",           z_threshold_);
  max_surface_distance_  = node_->declare_parameter(name_ + ".max_surface_distance",  max_surface_distance_);
  max_step_height_       = node_->declare_parameter(name_ + ".max_step_height",       max_step_height_);
  min_vertical_clearance_= node_->declare_parameter(name_ + ".min_vertical_clearance",min_vertical_clearance_);

  min_floor_support_ratio_  = node_->declare_parameter(name_ + ".min_floor_support_ratio",  min_floor_support_ratio_);
  floor_support_num_dirs_   = node_->declare_parameter(name_ + ".floor_support_num_dirs",   floor_support_num_dirs_);

  wall_proximity_height_    = node_->declare_parameter(name_ + ".wall_proximity_height",    wall_proximity_height_);
  wall_proximity_radius_    = node_->declare_parameter(name_ + ".wall_proximity_radius",    wall_proximity_radius_);
  wall_proximity_num_dirs_  = node_->declare_parameter(name_ + ".wall_proximity_num_dirs",  wall_proximity_num_dirs_);
  wall_proximity_num_z_     = node_->declare_parameter(name_ + ".wall_proximity_num_z",     wall_proximity_num_z_);

  walkability_recheck_passes_ = node_->declare_parameter(name_ + ".walkability_recheck_passes", walkability_recheck_passes_);
  walkability_recheck_detail_ = node_->declare_parameter(name_ + ".walkability_recheck_detail", walkability_recheck_detail_);

  enable_stair_edges_         = node_->declare_parameter(name_ + ".enable_stair_edges",         enable_stair_edges_);
  stair_xy_radius_            = node_->declare_parameter(name_ + ".stair_xy_radius",            stair_xy_radius_);
  stair_vertical_margin_      = node_->declare_parameter(name_ + ".stair_vertical_margin",      stair_vertical_margin_);
  stair_adjacency_multiplier_ = node_->declare_parameter(name_ + ".stair_adjacency_multiplier", stair_adjacency_multiplier_);
  max_bridge_dist_            = node_->declare_parameter(name_ + ".max_bridge_dist",            max_bridge_dist_);
  stair_min_rise_             = node_->declare_parameter(name_ + ".stair_min_rise",             stair_min_rise_);
  stair_max_rise_             = node_->declare_parameter(name_ + ".stair_max_rise",             stair_max_rise_);
  stair_max_xy_dist_          = node_->declare_parameter(name_ + ".stair_max_xy_dist",          stair_max_xy_dist_);
  stair_min_chain_length_     = node_->declare_parameter(name_ + ".stair_min_chain_length",     stair_min_chain_length_);
  stair_max_tread_thickness_  = node_->declare_parameter(name_ + ".stair_max_tread_thickness",  stair_max_tread_thickness_);

  // --- Penalty parameters ---------------------------------------------------
  wall_penalty_weight_    = node_->declare_parameter(name_ + ".wall_penalty_weight",    wall_penalty_weight_);
  corner_radius_          = node_->declare_parameter(name_ + ".corner_radius",          0.40);
  corner_penalty_weight_  = node_->declare_parameter(name_ + ".corner_penalty_weight",  corner_penalty_weight_);
  corner_angle_thresh_deg_= node_->declare_parameter(name_ + ".corner_angle_thresh_deg",corner_angle_thresh_deg_);
  sector_bins_            = node_->declare_parameter(name_ + ".sector_bins",            sector_bins_);
  sector_radius_          = node_->declare_parameter(name_ + ".sector_radius",          sector_radius_);
  sector_peak_thresh_     = node_->declare_parameter(name_ + ".sector_peak_thresh",     sector_peak_thresh_);
  sector_min_inliers_     = node_->declare_parameter(name_ + ".sector_min_inliers",     sector_min_inliers_);
  centroid_k_             = node_->declare_parameter(name_ + ".centroid_k",             centroid_k_);
  centroid_lambda_        = node_->declare_parameter(name_ + ".centroid_lambda",        centroid_lambda_);
  centroid_penalty_weight_= node_->declare_parameter(name_ + ".centroid_penalty_weight",centroid_penalty_weight_);
  ransac_radius_          = node_->declare_parameter(name_ + ".ransac_radius",          ransac_radius_);
  ransac_dist_thresh_     = node_->declare_parameter(name_ + ".ransac_dist_thresh",     ransac_dist_thresh_);
  ransac_min_inliers_     = node_->declare_parameter(name_ + ".ransac_min_inliers",     ransac_min_inliers_);
  ransac_iterations_      = node_->declare_parameter(name_ + ".ransac_iterations",      ransac_iterations_);
  penalty_spread_radius_  = node_->declare_parameter(name_ + ".penalty_spread_radius",  penalty_spread_radius_);
  penalty_spread_factor_  = node_->declare_parameter(name_ + ".penalty_spread_factor",  penalty_spread_factor_);
  worker_thread_limit_    = node_->declare_parameter(name_ + ".worker_thread_limit",    worker_thread_limit_);

  // --- Visualization --------------------------------------------------------
  publish_graph_markers_ = node_->declare_parameter(name_ + ".publish_graph_markers", publish_graph_markers_);
  graph_marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/graph_nodes", rclcpp::QoS(1).transient_local());

  // --- Parameter change callback -------------------------------------------
  reconfigure_handle_ = node_->add_on_set_parameters_callback(
    std::bind(&OctoMappingServer::reconfigureCallback, this, std::placeholders::_1));

  // --- Background graph-build timer ----------------------------------------
  graph_build_timer_ = node_->create_wall_timer(
    std::chrono::seconds(1),
    [this]() {
      if (!enable_octomap_updates_) return;
      if (!graph_dirty_) return;
      bool expected = false;
      if (!graph_building_.compare_exchange_strong(expected, true)) return;

      auto new_graph = std::make_shared<GraphData>();
      std::shared_ptr<GraphData> current_graph;
      {
        std::lock_guard<std::mutex> lock(graph_mutex_);
        current_graph = active_graph_;
      }

      if (incremental_graph_build_ && current_graph && !current_graph->empty()) {
        new_graph->nodes                      = current_graph->nodes;
        new_graph->adj                        = current_graph->adj;
        new_graph->node_penalty               = current_graph->node_penalty;
        new_graph->penalized_nodes            = current_graph->penalized_nodes;
        new_graph->node_cs_penalty            = current_graph->node_cs_penalty;
        new_graph->node_gb_penalty            = current_graph->node_gb_penalty;
        new_graph->penalty_computed_nodes     = current_graph->penalty_computed_nodes;
        new_graph->processed_occupied_keys    = current_graph->processed_occupied_keys;
        new_graph->nodes_needing_adjacency_update = current_graph->nodes_needing_adjacency_update;
        updateConnectivityGraphIncrementalInto(new_graph);
      } else {
        buildConnectivityGraphInto(new_graph);
      }

      {
        std::lock_guard<std::mutex> lock(graph_mutex_);
        active_graph_ = new_graph;
        processed_occupied_keys_          = new_graph->processed_occupied_keys;
        nodes_needing_adjacency_update_   = new_graph->nodes_needing_adjacency_update;
        graph_dirty_ = false;
      }

      if (publish_graph_markers_) {
        publishGraphMarkers(new_graph);
      }

      graph_building_ = false;
    }
  );

  // --- Shutdown callback: save octomap + graph on Ctrl+C ------------------
  rclcpp::on_shutdown([this]() {
    RCLCPP_INFO(node_->get_logger(), "OctoMappingServer: shutdown signal — saving maps...");
    saveMapsOnShutdown();
  });

  RCLCPP_INFO(node_->get_logger(),
    "OctoMappingServer initialized (name=%s, topic=%s)",
    name_.c_str(), octomap_topic_.c_str());
}

// =============================================================================
// OctoMapper interface
// =============================================================================

const octomap::OcTree * OctoMappingServer::getOctree() const
{
  return octree_.get();
}

double OctoMappingServer::getVoxelSize() const
{
  return active_voxel_size_;
}

std::string OctoMappingServer::getMapFrame() const
{
  return map_frame_;
}

void OctoMappingServer::getBounds(std::array<double, 3> & min_b,
                                   std::array<double, 3> & max_b) const
{
  min_b = min_bound_;
  max_b = max_bound_;
}

double OctoMappingServer::getMaxStepHeight() const
{
  return max_step_height_;
}

void OctoMappingServer::markPenaltiesDirty()
{
  penalties_dirty_ = true;
}

void OctoMappingServer::publishAdditionalMarkers(
  const visualization_msgs::msg::MarkerArray & ma)
{
  if (graph_marker_pub_) {
    graph_marker_pub_->publish(ma);
  }
}

// =============================================================================
// getReadyGraph — the main entry point for the planner
// =============================================================================

std::shared_ptr<GraphData> OctoMappingServer::getReadyGraph()
{
  // --- 1. Deep-copy the active graph (brief lock) ---------------------------
  std::shared_ptr<GraphData> planning_graph;
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    if (active_graph_) {
      planning_graph = std::make_shared<GraphData>(*active_graph_);
    }
  }

  // --- 2. Synchronous first build if no graph exists yet -------------------
  if (!planning_graph || planning_graph->empty()) {
    if (!octree_) {
      return nullptr;  // no map yet
    }
    RCLCPP_INFO(node_->get_logger(),
      "OctoMappingServer: no active graph — building synchronously (first call)...");
    auto new_graph = std::make_shared<GraphData>();
    buildConnectivityGraphInto(new_graph);
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      active_graph_ = new_graph;
      graph_dirty_ = false;
    }
    planning_graph = std::make_shared<GraphData>(*new_graph);
    RCLCPP_INFO(node_->get_logger(),
      "OctoMappingServer: synchronous build done (nodes=%zu)", planning_graph->size());
  }

  // --- 3. Walkability revalidation -----------------------------------------
  if (!planning_graph->nodes.empty() && walkability_recheck_detail_ > 0) {
    auto t0 = std::chrono::steady_clock::now();
    size_t reclassified = revalidateWalkability(
      planning_graph, walkability_recheck_passes_, walkability_recheck_detail_);
    double dur = std::chrono::duration_cast<std::chrono::duration<double>>(
      std::chrono::steady_clock::now() - t0).count();
    RCLCPP_INFO(node_->get_logger(),
      "Walkability revalidation: %.3f s, reclassified %zu nodes (passes=%d, detail=%d)",
      dur, reclassified, walkability_recheck_passes_, walkability_recheck_detail_);
  }

  // --- 4. Incremental penalty computation ----------------------------------
  if (!planning_graph->nodes.empty()) {
    computePendingPenalties(planning_graph);
  }

  return planning_graph;
}

// =============================================================================
// octomapCallback
// =============================================================================

void OctoMappingServer::octomapCallback(
  const octomap_msgs::msg::Octomap::SharedPtr msg)
{
  octomap::AbstractOcTree * tree_ptr = nullptr;
  if (msg->binary) {
    tree_ptr = octomap_msgs::binaryMsgToMap(*msg);
  } else {
    tree_ptr = octomap_msgs::fullMsgToMap(*msg);
  }
  if (!tree_ptr) {
    RCLCPP_ERROR(node_->get_logger(),
      "Failed to convert Octomap message (binary=%s)", msg->binary ? "true" : "false");
    return;
  }

  if (!enable_octomap_updates_) {
    delete tree_ptr;
    return;
  }

  octomap::OcTree * incoming_tree = dynamic_cast<octomap::OcTree *>(tree_ptr);
  if (!incoming_tree) {
    RCLCPP_ERROR(node_->get_logger(), "Octomap message is not an OcTree type");
    delete tree_ptr;
    return;
  }

  incoming_tree->setProbHit(octomap_prob_hit_);
  incoming_tree->setProbMiss(octomap_prob_miss_);
  incoming_tree->setOccupancyThres(octomap_thres_);
  incoming_tree->setClampingThresMin(octomap_clamp_min_);
  incoming_tree->setClampingThresMax(octomap_clamp_max_);

  map_frame_ = msg->header.frame_id;

  if (incremental_graph_build_ && octree_) {
    size_t merged_count = 0;
    for (auto it = incoming_tree->begin_leafs(), end = incoming_tree->end_leafs();
         it != end; ++it)
    {
      octomap::point3d coord = it.getCoordinate();
      float log_odds = it->getLogOdds();
      octomap::OcTreeNode * node = octree_->updateNode(coord, log_odds > 0);
      if (node) {
        node->setLogOdds(log_odds);
        ++merged_count;
      }
    }
    delete incoming_tree;
    RCLCPP_DEBUG(node_->get_logger(), "Merged %zu voxels into octree", merged_count);
  } else {
    octree_.reset(incoming_tree);
    processed_occupied_keys_.clear();
    nodes_needing_adjacency_update_.clear();
  }

  // Update bounds (never-shrink)
  active_voxel_size_ = octree_->getResolution();
  double expand = std::max(1.0, 5.0 * active_voxel_size_);

  double min_x = min_bound_[0], min_y = min_bound_[1], min_z = min_bound_[2];
  double max_x = max_bound_[0], max_y = max_bound_[1], max_z = max_bound_[2];

  bool first_init = (min_x == 0.0 && min_y == 0.0 && min_z == 0.0 &&
                     max_x == 0.0 && max_y == 0.0 && max_z == 0.0);
  if (first_init) {
    min_x = min_y = min_z =  std::numeric_limits<double>::infinity();
    max_x = max_y = max_z = -std::numeric_limits<double>::infinity();
  }

  size_t occupied = 0;
  size_t new_voxels = 0;
  for (auto it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it) {
    if (!octree_->isNodeOccupied(*it)) continue;
    auto c = it.getCoordinate();
    double cx = static_cast<double>(c.x());
    double cy = static_cast<double>(c.y());
    double cz = static_cast<double>(c.z());

    char key_buf[128];
    int ix_mm = static_cast<int>(std::round(cx * 1000.0));
    int iy_mm = static_cast<int>(std::round(cy * 1000.0));
    int iz_mm = static_cast<int>(std::round(cz * 1000.0));
    std::snprintf(key_buf, sizeof(key_buf), "occ_%d_%d_%d", ix_mm, iy_mm, iz_mm);
    std::string voxel_key(key_buf);

    if (processed_occupied_keys_.find(voxel_key) == processed_occupied_keys_.end()) {
      ++new_voxels;
      double node_z = cz;
      int node_ix_mm = static_cast<int>(std::round(cx * 1000.0));
      int node_iy_mm = static_cast<int>(std::round(cy * 1000.0));
      int node_iz_mm = static_cast<int>(std::round(node_z * 1000.0));
      char node_key_buf[128];
      std::snprintf(node_key_buf, sizeof(node_key_buf), "c_%d_%d_%d",
                    node_ix_mm, node_iy_mm, node_iz_mm);
      nodes_needing_adjacency_update_.insert(std::string(node_key_buf));
    }

    min_x = std::min(min_x, cx - expand);
    min_y = std::min(min_y, cy - expand);
    min_z = std::min(min_z, cz - expand);
    max_x = std::max(max_x, cx + expand);
    max_y = std::max(max_y, cy + expand);
    max_z = std::max(max_z, cz + expand);
    ++occupied;
  }

  if (occupied == 0 && first_init) {
    min_x = min_y = min_z = -1.0;
    max_x = max_y = max_z =  1.0;
    RCLCPP_WARN(node_->get_logger(), "No occupied leaves: using default small bounds.");
  }

  min_bound_ = {min_x, min_y, min_z};
  max_bound_ = {max_x, max_y, max_z};

  grid_size_x_ = static_cast<int>(std::ceil((max_x - min_x) / active_voxel_size_)) + 1;
  grid_size_y_ = static_cast<int>(std::ceil((max_y - min_y) / active_voxel_size_)) + 1;
  grid_size_z_ = static_cast<int>(std::ceil((max_z - min_z) / active_voxel_size_)) + 1;

  RCLCPP_DEBUG(node_->get_logger(),
    "Octomap update: frame=%s res=%.3f occupied=%zu new_voxels=%zu",
    map_frame_.c_str(), active_voxel_size_, occupied, new_voxels);

  graph_dirty_ = true;
}

// =============================================================================
// Graph building
// =============================================================================

void OctoMappingServer::buildConnectivityGraph(double /*eps*/)
{
  auto t_start = std::chrono::steady_clock::now();
  graph_nodes_.clear();
  graph_adj_.clear();
  if (!octree_) return;

  struct RawVoxel {
    octomap::point3d coord;
    octomap::OcTreeKey key;
    unsigned int depth;
    double node_size;
  };
  std::vector<RawVoxel> raw_voxels;
  raw_voxels.reserve(octree_->size());
  size_t total_leafs = 0;

  for (auto it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it) {
    ++total_leafs;
    if (!octree_->isNodeOccupied(*it)) continue;
    octomap::point3d occ = it.getCoordinate();
    if (!std::isfinite(occ.x()) || !std::isfinite(occ.y()) || !std::isfinite(occ.z())) continue;
    if (occ.x() < min_bound_[0] - 1e-6 || occ.x() > max_bound_[0] + 1e-6 ||
        occ.y() < min_bound_[1] - 1e-6 || occ.y() > max_bound_[1] + 1e-6 ||
        occ.z() < min_bound_[2] - 1e-6 || occ.z() > max_bound_[2] + 1e-6) continue;
    unsigned int depth = it.getDepth();
    raw_voxels.push_back({occ, octree_->coordToKey(occ), depth, octree_->getNodeSize(depth)});
  }

  RCLCPP_INFO(node_->get_logger(),
    "Graph build phase 1: collected %zu occupied voxels from %zu leafs",
    raw_voxels.size(), total_leafs);

  struct VoxelResult { std::string id; GraphNode gn; bool valid = false; };
  std::vector<VoxelResult> voxel_results(raw_voxels.size());
  std::atomic<size_t> skipped_collision{0};

  auto graph_build_parallel = [&](size_t count, auto && worker) {
    if (count == 0) return;
    unsigned int cap = (worker_thread_limit_ > 0)
      ? static_cast<unsigned int>(worker_thread_limit_)
      : std::max(2u, std::thread::hardware_concurrency());
    unsigned int num_threads = std::max(1u, std::min(cap, static_cast<unsigned int>(count)));
    RCLCPP_INFO(node_->get_logger(),
      "Graph build phase 2: using %u threads for %zu voxels", num_threads, count);
    if (num_threads <= 1) {
      worker(size_t(0), count);
    } else {
      size_t chunk = (count + num_threads - 1) / num_threads;
      std::vector<std::thread> threads;
      threads.reserve(num_threads);
      for (unsigned int t = 0; t < num_threads; ++t) {
        size_t s = t * chunk; if (s >= count) break;
        size_t e = std::min(s + chunk, count);
        threads.emplace_back(worker, s, e);
      }
      for (auto & th : threads) { if (th.joinable()) th.join(); }
    }
  };

  graph_build_parallel(raw_voxels.size(), [&](size_t v_start, size_t v_end) {
    for (size_t vi = v_start; vi < v_end; ++vi) {
      const auto & rv = raw_voxels[vi];
      const octomap::point3d & occ = rv.coord;
      double node_z = occ.z();

      bool vertical_clear = true;
      double stepz = std::max(active_voxel_size_, 0.05);
      double voxel_top = occ.z() + 0.5 * rv.node_size;
      for (double z = voxel_top + 0.01; z <= voxel_top + wall_proximity_height_; z += stepz) {
        octomap::OcTreeNode * nn = octree_->search(occ.x(), occ.y(), z);
        if (nn && octree_->isNodeOccupied(nn)) { vertical_clear = false; break; }
      }

      bool stair_rescued = false;
      if (!vertical_clear) {
        if (isStairStep(occ.x(), occ.y(), voxel_top, rv.node_size)) {
          vertical_clear = true;
          stair_rescued = true;
        } else {
          ++skipped_collision;
        }
      }

      if (vertical_clear) {
        if (!hasFloorSupport(occ.x(), occ.y(), occ.z(), rv.node_size)) {
          vertical_clear = false;
        }
      }

      octomap::point3d center(occ.x(), occ.y(), node_z);
      GraphNode gn;
      gn.key          = rv.key;
      gn.depth        = rv.depth;
      gn.center       = center;
      gn.size         = rv.node_size;
      gn.is_walkable  = vertical_clear;
      gn.is_stair_step = stair_rescued;

      char idbuf[128];
      int ix_mm = static_cast<int>(std::round(center.x() * 1000.0));
      int iy_mm = static_cast<int>(std::round(center.y() * 1000.0));
      int iz_mm = static_cast<int>(std::round(center.z() * 1000.0));
      std::snprintf(idbuf, sizeof(idbuf), "c_%d_%d_%d", ix_mm, iy_mm, iz_mm);

      voxel_results[vi].id = std::string(idbuf);
      voxel_results[vi].gn = gn;
      voxel_results[vi].valid = true;
    }
  });

  // Phase 3: insert into graph_nodes_
  for (const auto & vr : voxel_results) {
    if (!vr.valid) continue;
    graph_nodes_.emplace(vr.id, vr.gn);
  }

  size_t walkable_count = 0, stair_step_count = 0;
  for (const auto & [id, gn] : graph_nodes_) {
    if (gn.is_walkable)   ++walkable_count;
    if (gn.is_stair_step) ++stair_step_count;
  }
  RCLCPP_INFO(node_->get_logger(),
    "Graph build: %zu nodes (%zu walkable, %zu stair-step rescued)",
    graph_nodes_.size(), walkable_count, stair_step_count);

  // Phase 4: adjacency
  std::vector<std::pair<std::string, GraphNode>> node_snapshot;
  node_snapshot.reserve(graph_nodes_.size());
  for (const auto & kv : graph_nodes_) {
    node_snapshot.emplace_back(kv.first, kv.second);
  }

  auto pick_thread_count = [&](size_t workload) -> unsigned int {
    if (workload == 0) return 0u;
    unsigned int cap = (worker_thread_limit_ > 0)
      ? static_cast<unsigned int>(worker_thread_limit_)
      : std::max(2u, std::thread::hardware_concurrency());
    return std::max(1u, std::min(cap, static_cast<unsigned int>(workload)));
  };

  std::vector<std::vector<std::string>> adjacency(node_snapshot.size());
  auto add_edge = [&](size_t from_idx, const std::string & neighbor_id) {
    if (from_idx >= adjacency.size()) return;
    auto & vec = adjacency[from_idx];
    if (std::find(vec.begin(), vec.end(), neighbor_id) == vec.end())
      vec.push_back(neighbor_id);
  };

  auto neighbor_worker = [&](size_t start, size_t end) {
    for (size_t idx = start; idx < end; ++idx) {
      const GraphNode & node = node_snapshot[idx].second;
      double multiplier = node.is_stair_step ? stair_adjacency_multiplier_ : 1.8;
      double max_dist_sq = (node.size * multiplier) * (node.size * multiplier);
      for (size_t j = 0; j < node_snapshot.size(); ++j) {
        if (j == idx) continue;
        const GraphNode & other = node_snapshot[j].second;
        double dx = other.center.x() - node.center.x();
        double dy = other.center.y() - node.center.y();
        double dz = other.center.z() - node.center.z();
        double dist_sq = dx*dx + dy*dy + dz*dz;
        double effective_max_sq = max_dist_sq;
        if (other.is_stair_step && !node.is_stair_step) {
          double sm = other.size * stair_adjacency_multiplier_;
          effective_max_sq = sm * sm;
        }
        if (dist_sq <= effective_max_sq) {
          add_edge(idx, node_snapshot[j].first);
        }
      }
    }
  };

  if (!node_snapshot.empty()) {
    unsigned int num_threads = pick_thread_count(node_snapshot.size());
    if (num_threads == 0) num_threads = 1;
    if (num_threads == 1) {
      neighbor_worker(0, node_snapshot.size());
    } else {
      size_t chunk = (node_snapshot.size() + num_threads - 1) / num_threads;
      std::vector<std::thread> threads;
      threads.reserve(num_threads);
      for (unsigned int t = 0; t < num_threads; ++t) {
        size_t s = t * chunk; if (s >= node_snapshot.size()) break;
        size_t e = std::min(s + chunk, node_snapshot.size());
        threads.emplace_back(neighbor_worker, s, e);
      }
      for (auto & th : threads) { if (th.joinable()) th.join(); }
    }
    graph_adj_.clear();
    for (size_t idx = 0; idx < node_snapshot.size(); ++idx)
      graph_adj_[node_snapshot[idx].first] = std::move(adjacency[idx]);
  }

  // Phase 5: stair augmentation
  detectAndAugmentStairs();

  if (publish_graph_markers_) {
    publishGraphMarkers();
  }

  double dur = std::chrono::duration_cast<std::chrono::duration<double>>(
    std::chrono::steady_clock::now() - t_start).count();
  RCLCPP_INFO(node_->get_logger(),
    "buildConnectivityGraph took %.3f s (nodes=%zu)", dur, graph_nodes_.size());

  // Mark all occupied voxels as processed
  processed_occupied_keys_.clear();
  for (auto it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it) {
    if (octree_->isNodeOccupied(*it)) {
      auto c = it.getCoordinate();
      char key_buf[128];
      int ix_mm = static_cast<int>(std::round(c.x() * 1000.0));
      int iy_mm = static_cast<int>(std::round(c.y() * 1000.0));
      int iz_mm = static_cast<int>(std::round(c.z() * 1000.0));
      std::snprintf(key_buf, sizeof(key_buf), "occ_%d_%d_%d", ix_mm, iy_mm, iz_mm);
      processed_occupied_keys_.insert(std::string(key_buf));
    }
  }
  nodes_needing_adjacency_update_.clear();
}

void OctoMappingServer::buildConnectivityGraphInto(
  std::shared_ptr<GraphData> & target, double eps)
{
  buildConnectivityGraph(eps);
  target->nodes                        = graph_nodes_;
  target->adj                          = graph_adj_;
  target->node_penalty.clear();
  target->penalized_nodes.clear();
  target->processed_occupied_keys      = processed_occupied_keys_;
  target->nodes_needing_adjacency_update = nodes_needing_adjacency_update_;
}

// ---------------------------------------------------------------------------

void OctoMappingServer::updateConnectivityGraphIncremental(double /*eps*/)
{
  auto t_start = std::chrono::steady_clock::now();
  if (!octree_) return;

  using VoxelTuple = std::tuple<octomap::point3d, octomap::OcTreeKey, unsigned int, double>;
  std::vector<VoxelTuple> new_voxels;

  for (auto it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it) {
    if (!octree_->isNodeOccupied(*it)) continue;
    octomap::point3d occ = it.getCoordinate();
    char key_buf[128];
    int ix_mm = static_cast<int>(std::round(occ.x() * 1000.0));
    int iy_mm = static_cast<int>(std::round(occ.y() * 1000.0));
    int iz_mm = static_cast<int>(std::round(occ.z() * 1000.0));
    std::snprintf(key_buf, sizeof(key_buf), "occ_%d_%d_%d", ix_mm, iy_mm, iz_mm);
    std::string voxel_key(key_buf);

    if (processed_occupied_keys_.find(voxel_key) == processed_occupied_keys_.end()) {
      unsigned int depth = it.getDepth();
      double node_size = octree_->getNodeSize(depth);
      new_voxels.emplace_back(occ, octree_->coordToKey(occ), depth, node_size);
      processed_occupied_keys_.insert(voxel_key);
    }
  }

  if (new_voxels.empty()) {
    RCLCPP_DEBUG(node_->get_logger(), "Incremental update: no new voxels to process");
    return;
  }

  RCLCPP_INFO(node_->get_logger(),
    "Incremental update: processing %zu new voxels", new_voxels.size());

  std::vector<std::pair<std::string, GraphNode>> new_nodes;
  for (const auto & [occ, key, depth, node_size] : new_voxels) {
    double node_z = occ.z();
    if (!std::isfinite(occ.x()) || !std::isfinite(occ.y()) || !std::isfinite(node_z)) continue;
    if (occ.x() < min_bound_[0] - 1e-6 || occ.x() > max_bound_[0] + 1e-6 ||
        occ.y() < min_bound_[1] - 1e-6 || occ.y() > max_bound_[1] + 1e-6 ||
        node_z  < min_bound_[2] - 1e-6 || node_z  > max_bound_[2] + 1e-6) continue;

    bool vertical_clear = true;
    double stepz = std::max(active_voxel_size_, 0.05);
    double voxel_top = occ.z() + 0.5 * node_size;
    for (double z = voxel_top + 0.01; z <= voxel_top + wall_proximity_height_; z += stepz) {
      octomap::OcTreeNode * nn = octree_->search(occ.x(), occ.y(), z);
      if (nn && octree_->isNodeOccupied(nn)) { vertical_clear = false; break; }
    }

    bool stair_rescued = false;
    if (!vertical_clear) {
      if (isStairStep(occ.x(), occ.y(), voxel_top, node_size)) {
        vertical_clear = true;
        stair_rescued = true;
      }
    }

    if (vertical_clear) {
      if (!hasFloorSupport(occ.x(), occ.y(), occ.z(), node_size)) {
        vertical_clear = false;
      }
    }

    octomap::point3d center(occ.x(), occ.y(), node_z);
    GraphNode gn;
    gn.key          = octree_->coordToKey(center);
    gn.depth        = depth;
    gn.center       = center;
    gn.size         = node_size;
    gn.is_walkable  = vertical_clear;
    gn.is_stair_step = stair_rescued;

    char idbuf[128];
    int node_ix_mm = static_cast<int>(std::round(center.x() * 1000.0));
    int node_iy_mm = static_cast<int>(std::round(center.y() * 1000.0));
    int node_iz_mm = static_cast<int>(std::round(center.z() * 1000.0));
    std::snprintf(idbuf, sizeof(idbuf), "c_%d_%d_%d", node_ix_mm, node_iy_mm, node_iz_mm);
    std::string node_id(idbuf);

    if (graph_nodes_.find(node_id) == graph_nodes_.end()) {
      graph_nodes_.emplace(node_id, gn);
      graph_adj_[node_id] = {};
      new_nodes.emplace_back(node_id, gn);
      nodes_needing_adjacency_update_.insert(node_id);
    }
  }

  RCLCPP_INFO(node_->get_logger(),
    "Incremental update: added %zu new nodes (total=%zu)",
    new_nodes.size(), graph_nodes_.size());

  if (new_nodes.empty() && nodes_needing_adjacency_update_.empty()) return;

  // Expand adjacency update set to include existing nodes near new ones
  double neighbor_search_radius = active_voxel_size_ * 5.0;
  for (const auto & [new_id, new_node] : new_nodes) {
    for (const auto & [existing_id, existing_node] : graph_nodes_) {
      if (existing_id == new_id) continue;
      if (nodes_needing_adjacency_update_.count(existing_id)) continue;
      if (new_node.center.distance(existing_node.center) <= neighbor_search_radius) {
        nodes_needing_adjacency_update_.insert(existing_id);
      }
    }
  }

  // Build index over all nodes
  std::vector<std::pair<std::string, GraphNode>> all_nodes_snapshot;
  all_nodes_snapshot.reserve(graph_nodes_.size());
  for (const auto & kv : graph_nodes_) {
    all_nodes_snapshot.emplace_back(kv.first, kv.second);
  }

  // Collect nodes that need adjacency update
  std::vector<std::pair<std::string, GraphNode>> nodes_to_update;
  for (const auto & node_id : nodes_needing_adjacency_update_) {
    auto it = graph_nodes_.find(node_id);
    if (it != graph_nodes_.end()) nodes_to_update.emplace_back(it->first, it->second);
  }

  for (const auto & [node_id, node] : nodes_to_update) {
    auto & adj = graph_adj_[node_id];
    double inc_direct_mult = node.is_stair_step ? stair_adjacency_multiplier_ : 1.8;
    double max_dist_sq = (node.size * inc_direct_mult) * (node.size * inc_direct_mult);

    for (const auto & [cand_id, cand_node] : all_nodes_snapshot) {
      if (cand_id == node_id) continue;
      double dx = cand_node.center.x() - node.center.x();
      double dy = cand_node.center.y() - node.center.y();
      double dz = cand_node.center.z() - node.center.z();
      double dsq = dx*dx + dy*dy + dz*dz;
      double effective_max = max_dist_sq;
      if (cand_node.is_stair_step && !node.is_stair_step) {
        double sm = cand_node.size * stair_adjacency_multiplier_;
        effective_max = sm * sm;
      }
      if (dsq > effective_max) continue;
      if (std::find(adj.begin(), adj.end(), cand_id) == adj.end())
        adj.push_back(cand_id);
      auto & rev = graph_adj_[cand_id];
      if (std::find(rev.begin(), rev.end(), node_id) == rev.end())
        rev.push_back(node_id);
    }
  }

  nodes_needing_adjacency_update_.clear();

  if (publish_graph_markers_) {
    publishGraphMarkers();
  }

  double dur = std::chrono::duration_cast<std::chrono::duration<double>>(
    std::chrono::steady_clock::now() - t_start).count();
  RCLCPP_INFO(node_->get_logger(), "Incremental graph update took %.3f s", dur);
}

void OctoMappingServer::updateConnectivityGraphIncrementalInto(
  std::shared_ptr<GraphData> & target, double eps)
{
  processed_occupied_keys_          = target->processed_occupied_keys;
  nodes_needing_adjacency_update_   = target->nodes_needing_adjacency_update;
  graph_nodes_                      = target->nodes;
  graph_adj_                        = target->adj;

  updateConnectivityGraphIncremental(eps);

  target->nodes                         = graph_nodes_;
  target->adj                           = graph_adj_;
  target->processed_occupied_keys       = processed_occupied_keys_;
  target->nodes_needing_adjacency_update = nodes_needing_adjacency_update_;
}

// =============================================================================
// Voxel classification helpers
// =============================================================================

bool OctoMappingServer::hasFloorSupport(double x, double y, double z,
                                         double node_size) const
{
  if (!octree_) return false;
  const double search_radius = std::max(
    static_cast<double>(0.1) /* robot_radius placeholder */, node_size * 2.0);
  const double z_tol  = std::max(active_voxel_size_ * 2.0, 0.15);
  const double step_z = std::max(active_voxel_size_, 0.05);
  const int num_dirs  = std::max(4, floor_support_num_dirs_);
  int support_count   = 0;

  for (int d = 0; d < num_dirs; ++d) {
    double angle = d * (2.0 * M_PI / num_dirs);
    double dx = search_radius * std::cos(angle);
    double dy = search_radius * std::sin(angle);
    bool found = false;
    for (double dz = -z_tol; dz <= z_tol; dz += step_z) {
      octomap::OcTreeNode * nn = octree_->search(x + dx, y + dy, z + dz);
      if (nn && octree_->isNodeOccupied(nn)) { found = true; break; }
    }
    if (found) ++support_count;
  }
  return static_cast<double>(support_count) / num_dirs >= min_floor_support_ratio_;
}

bool OctoMappingServer::isStairStep(double x, double y, double voxel_top,
                                     double node_size) const
{
  if (!octree_) return false;
  const double min_rise = stair_min_rise_;
  const double max_rise = std::max(stair_max_rise_, max_step_height_);
  const double step_z   = std::max(active_voxel_size_, 0.05);

  double blocker_z = -1.0;
  for (double z = voxel_top + 0.01; z <= voxel_top + wall_proximity_height_; z += step_z) {
    octomap::OcTreeNode * nn = octree_->search(x, y, z);
    if (nn && octree_->isNodeOccupied(nn)) { blocker_z = z; break; }
  }
  if (blocker_z < 0.0) return false;

  double rise = blocker_z - voxel_top;
  if (rise < min_rise || rise > max_rise) return false;
  if (!hasFloorSupport(x, y, blocker_z, node_size)) return false;

  double blocker_top     = blocker_z + 0.5 * active_voxel_size_;
  double clearance_needed = std::max(min_vertical_clearance_, wall_proximity_height_ * 0.5);
  for (double z = blocker_top + 0.01; z <= blocker_top + clearance_needed; z += step_z) {
    octomap::OcTreeNode * nn = octree_->search(x, y, z);
    if (nn && octree_->isNodeOccupied(nn)) return false;
  }
  return true;
}

bool OctoMappingServer::hasVerticalClearance(const octomap::point3d & center,
                                              double node_size,
                                              double check_height) const
{
  if (!octree_) return true;
  const double stepz     = std::max(active_voxel_size_, 0.05);
  const double voxel_top = center.z() + 0.5 * node_size;
  for (double z = voxel_top + 0.01; z <= voxel_top + check_height; z += stepz) {
    octomap::OcTreeNode * nn = octree_->search(center.x(), center.y(), z);
    if (nn && octree_->isNodeOccupied(nn)) return false;
  }
  return true;
}

// =============================================================================
// Stair detection
// =============================================================================

void OctoMappingServer::detectAndAugmentStairs()
{
  if (!enable_stair_edges_ || !octree_) return;

  const double voxel_sz = std::max(active_voxel_size_, 0.05);

  struct TreadCandidate { std::string id; double x, y, z; };
  std::vector<TreadCandidate> candidates;
  candidates.reserve(graph_nodes_.size() / 2);

  for (auto & [id, node] : graph_nodes_) {
    if (!hasFloorSupport(node.center.x(), node.center.y(), node.center.z(), node.size))
      continue;

    double cx = node.center.x(), cy = node.center.y(), cz = node.center.z();
    double thickness_below = 0.0;
    for (double z = cz - voxel_sz; z >= cz - 1.0; z -= voxel_sz) {
      octomap::OcTreeNode * nn = octree_->search(cx, cy, z);
      if (nn && octree_->isNodeOccupied(nn)) thickness_below += voxel_sz; else break;
    }
    double thickness_above = 0.0;
    for (double z = cz + voxel_sz; z <= cz + 1.0; z += voxel_sz) {
      octomap::OcTreeNode * nn = octree_->search(cx, cy, z);
      if (nn && octree_->isNodeOccupied(nn)) thickness_above += voxel_sz; else break;
    }
    if (thickness_below + voxel_sz + thickness_above > stair_max_tread_thickness_) continue;

    candidates.push_back({id, cx, cy, cz});
  }

  if (candidates.size() < static_cast<size_t>(stair_min_chain_length_)) return;

  std::unordered_map<std::string, std::vector<std::string>> stair_adj;
  for (size_t i = 0; i < candidates.size(); ++i) {
    for (size_t j = i + 1; j < candidates.size(); ++j) {
      double dz  = std::abs(candidates[j].z - candidates[i].z);
      if (dz < stair_min_rise_ || dz > stair_max_rise_) continue;
      double dxy = std::hypot(candidates[j].x - candidates[i].x,
                              candidates[j].y - candidates[i].y);
      if (dxy > stair_max_xy_dist_) continue;
      stair_adj[candidates[i].id].push_back(candidates[j].id);
      stair_adj[candidates[j].id].push_back(candidates[i].id);
    }
  }

  std::unordered_set<std::string> visited;
  size_t total_stair_nodes = 0, stair_regions = 0;

  for (const auto & cand : candidates) {
    if (visited.count(cand.id)) continue;
    if (!stair_adj.count(cand.id)) continue;

    std::vector<std::string> component;
    std::queue<std::string> bfs_q;
    bfs_q.push(cand.id);
    visited.insert(cand.id);
    while (!bfs_q.empty()) {
      std::string u = bfs_q.front(); bfs_q.pop();
      component.push_back(u);
      for (const auto & v : stair_adj[u]) {
        if (visited.insert(v).second) bfs_q.push(v);
      }
    }

    if (static_cast<int>(component.size()) < stair_min_chain_length_) continue;
    ++stair_regions;
    total_stair_nodes += component.size();

    for (const auto & id : component) {
      auto it = graph_nodes_.find(id);
      if (it != graph_nodes_.end()) {
        it->second.is_walkable  = true;
        it->second.is_stair_step = true;
      }
    }
    for (const auto & id : component) {
      auto sa_it = stair_adj.find(id);
      if (sa_it == stair_adj.end()) continue;
      for (const auto & nb : sa_it->second) {
        auto & adj = graph_adj_[id];
        if (std::find(adj.begin(), adj.end(), nb) == adj.end()) adj.push_back(nb);
        auto & rev = graph_adj_[nb];
        if (std::find(rev.begin(), rev.end(), id) == rev.end()) rev.push_back(id);
      }
    }
  }

  if (total_stair_nodes > 0) {
    RCLCPP_INFO(node_->get_logger(),
      "Stair detection: %zu candidates, %zu regions, %zu tread nodes promoted",
      candidates.size(), stair_regions, total_stair_nodes);
  }
}

// =============================================================================
// Walkability revalidation
// =============================================================================

size_t OctoMappingServer::revalidateWalkability(
  std::shared_ptr<GraphData> & graph, int max_passes, int detail_level)
{
  if (!graph || !octree_ || detail_level <= 0) return 0;

  size_t total_reclassified = 0;

  for (int pass = 0; pass < std::max(1, max_passes); ++pass) {
    size_t reclassified_this_pass = 0;
    for (auto & kv : graph->nodes) {
      if (!kv.second.is_walkable) continue;
      bool vc_ok = hasVerticalClearance(kv.second.center, kv.second.size,
                                         wall_proximity_height_);
      bool fs_ok = vc_ok && hasFloorSupport(kv.second.center.x(), kv.second.center.y(),
                                             kv.second.center.z(), kv.second.size);
      if (!vc_ok || !fs_ok) {
        kv.second.is_walkable = false;
        ++reclassified_this_pass;
        graph->penalty_computed_nodes.erase(kv.first);
        graph->node_cs_penalty.erase(kv.first);
        graph->node_gb_penalty.erase(kv.first);
        graph->node_penalty.erase(kv.first);
        graph->penalized_nodes.erase(kv.first);
      }
    }
    total_reclassified += reclassified_this_pass;
    RCLCPP_INFO(node_->get_logger(),
      "Walkability pass %d/%d (detail=%d): reclassified %zu nodes",
      pass + 1, max_passes, detail_level, reclassified_this_pass);
    if (reclassified_this_pass == 0) break;
  }

  if (total_reclassified > 0) {
    graph->penalty_computed_nodes.clear();
    graph->node_cs_penalty.clear();
    graph->node_gb_penalty.clear();
    graph->node_penalty.clear();
    graph->penalized_nodes.clear();
    RCLCPP_INFO(node_->get_logger(),
      "Revalidation: %zu nodes reclassified — full penalty cache invalidated",
      total_reclassified);
  }

  if (detail_level >= 2) {
    const double scan_radius = wall_proximity_radius_;
    const int num_dirs = std::max(4, wall_proximity_num_dirs_);
    const int num_z    = std::max(1, wall_proximity_num_z_);
    const double angle_step = 2.0 * M_PI / static_cast<double>(num_dirs);
    size_t wall_adjacent_invalidated = 0;

    for (auto & kv : graph->nodes) {
      if (!kv.second.is_walkable) continue;
      if (!graph->penalty_computed_nodes.count(kv.first)) continue;

      double voxel_top = kv.second.center.z() + 0.5 * kv.second.size;
      bool found_wall = false;
      for (int iz = 0; iz < num_z && !found_wall; ++iz) {
        double frac = (num_z == 1) ? 0.5 :
          static_cast<double>(iz) / static_cast<double>(num_z - 1);
        double z = voxel_top + 0.02 + frac * wall_proximity_height_;
        for (int idir = 0; idir < num_dirs && !found_wall; ++idir) {
          double angle = idir * angle_step;
          double sx = kv.second.center.x() + scan_radius * std::cos(angle);
          double sy = kv.second.center.y() + scan_radius * std::sin(angle);
          octomap::OcTreeNode * nn = octree_->search(sx, sy, z);
          if (nn && octree_->isNodeOccupied(nn)) found_wall = true;
        }
      }
      if (found_wall) {
        graph->penalty_computed_nodes.erase(kv.first);
        graph->node_cs_penalty.erase(kv.first);
        graph->node_gb_penalty.erase(kv.first);
        graph->node_penalty.erase(kv.first);
        ++wall_adjacent_invalidated;
      }
    }
    RCLCPP_INFO(node_->get_logger(),
      "Revalidation detail=%d: wall-proximity scan invalidated %zu penalty caches",
      detail_level, wall_adjacent_invalidated);
  }

  if (detail_level >= 3) {
    size_t cached_before = graph->penalty_computed_nodes.size();
    graph->penalty_computed_nodes.clear();
    graph->node_cs_penalty.clear();
    graph->node_gb_penalty.clear();
    graph->node_penalty.clear();
    graph->penalized_nodes.clear();
    RCLCPP_INFO(node_->get_logger(),
      "Revalidation detail=3: force-invalidated %zu penalty cache entries", cached_before);
  }

  return total_reclassified;
}

// =============================================================================
// Penalty / costmap computation
// =============================================================================

void OctoMappingServer::computePendingPenalties(std::shared_ptr<GraphData> & graph)
{
  bool force_full = penalties_dirty_.load();
  if (force_full) {
    graph->penalty_computed_nodes.clear();
    graph->node_cs_penalty.clear();
    graph->node_gb_penalty.clear();
    graph->node_penalty.clear();
    graph->penalized_nodes.clear();
  }

  // Collect walkable nodes not yet in the cached costmap
  std::unordered_set<std::string> nodes_to_compute;
  for (const auto & kv : graph->nodes) {
    if (kv.second.is_walkable && !graph->penalty_computed_nodes.count(kv.first))
      nodes_to_compute.insert(kv.first);
  }

  if (nodes_to_compute.empty()) {
    RCLCPP_INFO(node_->get_logger(),
      "All %zu walkable nodes have cached penalties — skipping recomputation.",
      graph->penalty_computed_nodes.size());
    if (publish_graph_markers_ && graph_marker_pub_) {
      publishGraphMarkers(graph);
    }
    // Assign max penalty to non-walkable nodes
    for (const auto & kv : graph->nodes) {
      if (!kv.second.is_walkable) {
        graph->node_penalty[kv.first] = wall_penalty_weight_;
        graph->penalized_nodes.insert(kv.first);
      }
    }
    return;
  }

  RCLCPP_INFO(node_->get_logger(),
    "Computing penalties for %zu new nodes (%zu cached, %s)...",
    nodes_to_compute.size(), graph->penalty_computed_nodes.size(),
    force_full ? "full recompute" : "incremental");
  auto t_pen_start = std::chrono::steady_clock::now();

  // ---- Spatial hash of walkable nodes for centroid-shift ------------------
  const double cell_size = active_voxel_size_ * 2.0;
  struct GridKey { int gx, gy;
    bool operator==(const GridKey & o) const { return gx == o.gx && gy == o.gy; } };
  struct GridHash { size_t operator()(const GridKey & k) const {
    return std::hash<int>()(k.gx) ^ (std::hash<int>()(k.gy) << 16); } };
  std::unordered_map<GridKey, std::vector<size_t>, GridHash> walkable_grid;

  struct WalkableEntry { std::string id; octomap::point3d center; double z; };
  std::vector<WalkableEntry> walkable_list;
  walkable_list.reserve(graph->nodes.size());
  std::unordered_map<std::string, size_t> id_to_idx;

  for (const auto & kv : graph->nodes) {
    if (!kv.second.is_walkable) continue;
    size_t idx = walkable_list.size();
    walkable_list.push_back({kv.first, kv.second.center,
                              static_cast<double>(kv.second.center.z())});
    id_to_idx[kv.first] = idx;
    int gx = static_cast<int>(std::floor(kv.second.center.x() / cell_size));
    int gy = static_cast<int>(std::floor(kv.second.center.y() / cell_size));
    walkable_grid[{gx, gy}].push_back(idx);
  }

  std::vector<bool> needs_compute(walkable_list.size(), false);
  for (size_t wi = 0; wi < walkable_list.size(); ++wi) {
    if (nodes_to_compute.count(walkable_list[wi].id)) {
      needs_compute[wi] = true;
    }
  }

  // ---- Thread helper -------------------------------------------------------
  auto parallel_for = [&](size_t count, auto && worker) {
    if (count == 0) return;
    unsigned int cap = (worker_thread_limit_ > 0)
      ? static_cast<unsigned int>(worker_thread_limit_)
      : std::max(2u, std::thread::hardware_concurrency());
    unsigned int num_threads = std::max(1u, std::min(cap, static_cast<unsigned int>(count)));
    if (num_threads <= 1) {
      worker(size_t(0), count);
    } else {
      size_t chunk = (count + num_threads - 1) / num_threads;
      std::vector<std::thread> threads;
      threads.reserve(num_threads);
      for (unsigned int t = 0; t < num_threads; ++t) {
        size_t s = t * chunk; if (s >= count) break;
        size_t e = std::min(s + chunk, count);
        threads.emplace_back(worker, s, e);
      }
      for (auto & th : threads) { if (th.joinable()) th.join(); }
    }
  };

  // ---- Part 1: Centroid-shift computation ---------------------------------
  const double search_radius = std::max(penalty_spread_radius_, active_voxel_size_ * 3.0);
  const double z_tol         = active_voxel_size_ * 1.0;
  const int search_cells     = static_cast<int>(std::ceil(search_radius / cell_size)) + 1;
  std::atomic<size_t> edge_count{0};

  struct ShiftInfo { double shift_xy = 0.0; double shift_x = 0.0; double shift_y = 0.0; int n_neighbors = 0; };
  std::vector<ShiftInfo> shift_data(walkable_list.size());

  parallel_for(walkable_list.size(), [&](size_t w_start, size_t w_end) {
    for (size_t wi = w_start; wi < w_end; ++wi) {
      if (!needs_compute[wi]) continue;
      const auto & w = walkable_list[wi];
      int gx = static_cast<int>(std::floor(w.center.x() / cell_size));
      int gy = static_cast<int>(std::floor(w.center.y() / cell_size));
      double sum_x = 0.0, sum_y = 0.0;
      int count = 0;
      for (int dgx = -search_cells; dgx <= search_cells; ++dgx) {
        for (int dgy = -search_cells; dgy <= search_cells; ++dgy) {
          auto it = walkable_grid.find({gx + dgx, gy + dgy});
          if (it == walkable_grid.end()) continue;
          for (size_t nidx : it->second) {
            if (nidx == wi) continue;
            const auto & nb = walkable_list[nidx];
            if (std::abs(nb.z - w.z) > z_tol) continue;
            double dx = nb.center.x() - w.center.x();
            double dy = nb.center.y() - w.center.y();
            if (std::sqrt(dx*dx + dy*dy) <= search_radius) {
              sum_x += nb.center.x(); sum_y += nb.center.y(); ++count;
            }
          }
        }
      }
      if (count >= 2) {
        double sx = sum_x / count - w.center.x();
        double sy = sum_y / count - w.center.y();
        shift_data[wi] = {std::sqrt(sx*sx + sy*sy), sx, sy, count};
      }
    }
  });

  // ---- Part 2: Compute cs + gb penalties per node -------------------------
  const double norm_factor = search_radius * 0.75;
  std::atomic<size_t> graph_border_count{0};
  std::vector<double> cs_penalties(walkable_list.size(), 0.0);
  std::vector<double> gb_penalties(walkable_list.size(), 0.0);

  for (size_t wi = 0; wi < walkable_list.size(); ++wi) {
    if (!needs_compute[wi]) {
      auto cs_it = graph->node_cs_penalty.find(walkable_list[wi].id);
      if (cs_it != graph->node_cs_penalty.end()) cs_penalties[wi] = cs_it->second;
      auto gb_it = graph->node_gb_penalty.find(walkable_list[wi].id);
      if (gb_it != graph->node_gb_penalty.end()) gb_penalties[wi] = gb_it->second;
    }
  }

  parallel_for(walkable_list.size(), [&](size_t w_start, size_t w_end) {
    for (size_t wi = w_start; wi < w_end; ++wi) {
      if (!needs_compute[wi]) continue;
      const auto & w  = walkable_list[wi];
      const auto & si = shift_data[wi];

      // Centroid-shift penalty
      double cs_penalty = 0.0;
      if (si.n_neighbors >= 2 && si.shift_xy > 1e-6) {
        double ratio = std::min(1.0, si.shift_xy / norm_factor);
        cs_penalty = wall_penalty_weight_ * ratio * ratio;
        if (ratio > 0.3) ++edge_count;
        if (ratio > 0.2 && si.n_neighbors >= 4) {
          int gx = static_cast<int>(std::floor(w.center.x() / cell_size));
          int gy = static_cast<int>(std::floor(w.center.y() / cell_size));
          double shift_angle = std::atan2(si.shift_y, si.shift_x);
          int quadrant_mask = 0;
          for (int dgx = -search_cells; dgx <= search_cells; ++dgx) {
            for (int dgy = -search_cells; dgy <= search_cells; ++dgy) {
              auto it = walkable_grid.find({gx + dgx, gy + dgy});
              if (it == walkable_grid.end()) continue;
              for (size_t nidx : it->second) {
                if (nidx == wi) continue;
                const auto & nb = walkable_list[nidx];
                if (std::abs(nb.z - w.z) > z_tol) continue;
                double dx = nb.center.x() - w.center.x();
                double dy = nb.center.y() - w.center.y();
                if (dx*dx + dy*dy > search_radius * search_radius) continue;
                double angle = std::atan2(dy, dx) - shift_angle;
                while (angle >  M_PI) angle -= 2.0 * M_PI;
                while (angle < -M_PI) angle += 2.0 * M_PI;
                int q;
                if      (angle >= -M_PI/4 && angle <  M_PI/4) q = 0;
                else if (angle >=  M_PI/4 && angle < 3*M_PI/4) q = 1;
                else if (angle >= -3*M_PI/4 && angle < -M_PI/4) q = 3;
                else q = 2;
                quadrant_mask |= (1 << q);
              }
            }
          }
          int n_quadrants = __builtin_popcount(quadrant_mask);
          if (n_quadrants <= 2 && ratio > 0.4) {
            double corner_factor = (3.0 - n_quadrants) / 2.0;
            cs_penalty += corner_penalty_weight_ * ratio * corner_factor;
          }
        }
      }

      // Graph-border penalty
      double gb_penalty = 0.0;
      {
        auto adj_it = graph->adj.find(w.id);
        int octant_mask = 0;
        int walkable_neighbor_count = 0;
        if (adj_it != graph->adj.end()) {
          for (const auto & nb_id : adj_it->second) {
            auto nb_it = graph->nodes.find(nb_id);
            if (nb_it == graph->nodes.end() || !nb_it->second.is_walkable) continue;
            ++walkable_neighbor_count;
            double dx = nb_it->second.center.x() - w.center.x();
            double dy = nb_it->second.center.y() - w.center.y();
            int octant = static_cast<int>(
              std::floor((std::atan2(dy, dx) + M_PI) / (M_PI / 4.0))) % 8;
            octant_mask |= (1 << octant);
          }
        }
        int covered = __builtin_popcount(octant_mask);
        if (covered < 7 && walkable_neighbor_count >= 1) {
          int missing = 8 - covered;
          double border_ratio = static_cast<double>(missing) / 8.0;
          gb_penalty = wall_penalty_weight_ * border_ratio;
          if (missing >= 4) gb_penalty += corner_penalty_weight_ * 0.5 * border_ratio;
          ++graph_border_count;
        } else if (walkable_neighbor_count == 0) {
          gb_penalty = wall_penalty_weight_;
        }
      }

      cs_penalties[wi] = cs_penalty;
      gb_penalties[wi] = gb_penalty;
    }
  });

  // ---- Part 3: Spread gb penalty through graph edges ----------------------
  const double spread_radius = penalty_spread_radius_;
  const double spread_decay  = penalty_spread_factor_;
  std::vector<double> spread_gb(walkable_list.size(), 0.0);
  {
    struct SpreadEntry { double dist; size_t wi; double source_gb; };
    auto cmp = [](const SpreadEntry & a, const SpreadEntry & b) { return a.dist > b.dist; };
    std::priority_queue<SpreadEntry, std::vector<SpreadEntry>, decltype(cmp)> pq(cmp);
    std::vector<double> best_dist(walkable_list.size(), std::numeric_limits<double>::max());

    for (size_t wi = 0; wi < walkable_list.size(); ++wi) {
      if (gb_penalties[wi] > 1e-9) {
        spread_gb[wi] = gb_penalties[wi];
        best_dist[wi] = 0.0;
        pq.push({0.0, wi, gb_penalties[wi]});
      }
    }
    while (!pq.empty()) {
      auto top = pq.top(); pq.pop();
      if (top.dist > best_dist[top.wi] + 1e-9) continue;
      auto adj_it = graph->adj.find(walkable_list[top.wi].id);
      if (adj_it == graph->adj.end()) continue;
      for (const auto & nb_id : adj_it->second) {
        auto idx_it = id_to_idx.find(nb_id);
        if (idx_it == id_to_idx.end()) continue;
        size_t nb_wi = idx_it->second;
        double edge_len = walkable_list[top.wi].center.distance(walkable_list[nb_wi].center);
        double new_dist = top.dist + edge_len;
        if (new_dist >= spread_radius) continue;
        double new_val = top.source_gb * (1.0 - new_dist / spread_radius) * spread_decay;
        if (new_val > spread_gb[nb_wi] + 1e-9) {
          spread_gb[nb_wi] = new_val;
          best_dist[nb_wi] = new_dist;
          pq.push({new_dist, nb_wi, top.source_gb});
        }
      }
    }
  }

  // ---- Part 4: Final penalty = max(cs, gb, spread_gb) --------------------
  size_t penalized_count = 0;
  for (size_t wi = 0; wi < walkable_list.size(); ++wi) {
    double penalty = std::max({cs_penalties[wi], gb_penalties[wi], spread_gb[wi]});
    graph->node_penalty[walkable_list[wi].id] = penalty;
    if (penalty > 1e-9) { graph->penalized_nodes.insert(walkable_list[wi].id); ++penalized_count; }
  }
  // Cache raw values for newly computed nodes
  for (size_t wi = 0; wi < walkable_list.size(); ++wi) {
    if (needs_compute[wi]) {
      graph->node_cs_penalty[walkable_list[wi].id] = cs_penalties[wi];
      graph->node_gb_penalty[walkable_list[wi].id] = gb_penalties[wi];
      graph->penalty_computed_nodes.insert(walkable_list[wi].id);
    }
  }
  // Non-walkable: always max penalty
  for (const auto & kv : graph->nodes) {
    if (!kv.second.is_walkable) {
      graph->node_penalty[kv.first] = wall_penalty_weight_;
      graph->penalized_nodes.insert(kv.first);
    }
  }

  double dur_pen = std::chrono::duration_cast<std::chrono::duration<double>>(
    std::chrono::steady_clock::now() - t_pen_start).count();
  RCLCPP_INFO(node_->get_logger(),
    "Penalty: %.3f s, walkable=%zu, new=%zu, cs_edges=%zu, borders=%zu, penalized=%zu",
    dur_pen, walkable_list.size(), nodes_to_compute.size(),
    edge_count.load(), graph_border_count.load(), penalized_count);

  // Persist updated penalties back to active_graph_
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    if (active_graph_) {
      active_graph_->node_penalty           = graph->node_penalty;
      active_graph_->penalized_nodes        = graph->penalized_nodes;
      active_graph_->node_cs_penalty        = graph->node_cs_penalty;
      active_graph_->node_gb_penalty        = graph->node_gb_penalty;
      active_graph_->penalty_computed_nodes = graph->penalty_computed_nodes;
    }
  }
  penalties_dirty_ = false;

  if (publish_graph_markers_ && graph_marker_pub_) {
    publishGraphMarkers(graph);
  }
}

// =============================================================================
// Visualization
// =============================================================================

void OctoMappingServer::publishGraphMarkers()
{
  if (!graph_marker_pub_ || graph_nodes_.empty()) return;
  visualization_msgs::msg::MarkerArray ma;
  visualization_msgs::msg::Marker m;
  m.header.frame_id = map_frame_.empty() ? "map" : map_frame_;
  m.header.stamp    = node_->now();
  m.ns    = "graph_nodes";
  m.id    = 0;
  m.type  = visualization_msgs::msg::Marker::CUBE_LIST;
  m.action= visualization_msgs::msg::Marker::ADD;
  double scale = std::max(active_voxel_size_, 0.05);
  m.scale.x = static_cast<float>(scale);
  m.scale.y = static_cast<float>(scale);
  m.scale.z = static_cast<float>(scale);
  m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.0f; m.color.a = 0.8f;

  for (const auto & kv : graph_nodes_) {
    geometry_msgs::msg::Point p;
    p.x = kv.second.center.x();
    p.y = kv.second.center.y();
    p.z = kv.second.center.z();
    m.points.push_back(p);
    std_msgs::msg::ColorRGBA c;
    if (kv.second.is_walkable) { c.r = 0.0f; c.g = 1.0f; c.b = 0.0f; c.a = 0.8f; }
    else                       { c.r = 1.0f; c.g = 0.0f; c.b = 0.0f; c.a = 0.8f; }
    m.colors.push_back(c);
  }
  ma.markers.push_back(m);
  graph_marker_pub_->publish(ma);
}

void OctoMappingServer::publishGraphMarkers(
  const std::shared_ptr<GraphData> & graph)
{
  if (!graph_marker_pub_ || !graph || graph->nodes.empty()) return;
  visualization_msgs::msg::MarkerArray ma;

  // Node structure marker (green = walkable, red = non-walkable)
  visualization_msgs::msg::Marker m;
  m.header.frame_id = map_frame_.empty() ? "map" : map_frame_;
  m.header.stamp    = node_->now();
  m.ns    = "graph_nodes";
  m.id    = 0;
  m.type  = visualization_msgs::msg::Marker::CUBE_LIST;
  m.action= visualization_msgs::msg::Marker::ADD;
  double scale = std::max(active_voxel_size_, 0.05);
  m.scale.x = static_cast<float>(scale);
  m.scale.y = static_cast<float>(scale);
  m.scale.z = static_cast<float>(scale);
  m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.0f; m.color.a = 0.8f;

  for (const auto & kv : graph->nodes) {
    geometry_msgs::msg::Point p;
    p.x = kv.second.center.x();
    p.y = kv.second.center.y();
    p.z = kv.second.center.z();
    m.points.push_back(p);
    std_msgs::msg::ColorRGBA c;
    if (kv.second.is_walkable) { c.r = 0.0f; c.g = 1.0f; c.b = 0.0f; c.a = 0.8f; }
    else                       { c.r = 1.0f; c.g = 0.0f; c.b = 0.0f; c.a = 0.8f; }
    m.colors.push_back(c);
  }
  ma.markers.push_back(m);

  // Penalty gradient marker
  visualization_msgs::msg::Marker pen_m;
  pen_m.header = m.header;
  pen_m.ns     = "graph_penalty";
  pen_m.id     = 3;
  pen_m.type   = visualization_msgs::msg::Marker::CUBE_LIST;
  pen_m.action = visualization_msgs::msg::Marker::ADD;
  pen_m.scale.x = static_cast<float>(scale);
  pen_m.scale.y = static_cast<float>(scale);
  pen_m.scale.z = static_cast<float>(scale);

  for (const auto & kv : graph->nodes) {
    geometry_msgs::msg::Point p;
    p.x = kv.second.center.x();
    p.y = kv.second.center.y();
    p.z = kv.second.center.z();
    pen_m.points.push_back(p);
    double pen = 0.0;
    if (!kv.second.is_walkable) {
      pen = wall_penalty_weight_;
    } else {
      auto pit = graph->node_penalty.find(kv.first);
      if (pit != graph->node_penalty.end()) pen = pit->second;
    }
    std_msgs::msg::ColorRGBA c;
    double v = std::min(1.0, pen / std::max(1e-6, wall_penalty_weight_));
    c.r = static_cast<float>(v);
    c.g = static_cast<float>(1.0 - v);
    c.b = 0.0f; c.a = 0.9f;
    pen_m.colors.push_back(c);
  }
  if (!pen_m.points.empty()) ma.markers.push_back(pen_m);

  graph_marker_pub_->publish(ma);
  RCLCPP_DEBUG(node_->get_logger(),
    "Published graph markers: nodes=%zu, penalty_nodes=%zu",
    m.points.size(), pen_m.points.size());
}

// =============================================================================
// Map persistence
// =============================================================================

void OctoMappingServer::saveMapsOnShutdown()
{
  bool expected = false;
  if (!maps_saved_.compare_exchange_strong(expected, true)) return;

  try {
    auto now = std::chrono::system_clock::now();
    std::time_t now_t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf{};
    localtime_r(&now_t, &tm_buf);
    std::ostringstream ts;
    ts << std::put_time(&tm_buf, "%Y-%m-%d_%H-%M-%S");
    std::string timestamp = ts.str();

    const char * home = std::getenv("HOME");
    if (!home) home = "/tmp";
    std::filesystem::path save_dir = std::filesystem::path(home) / "alert_maps";
    std::filesystem::create_directories(save_dir);

    if (octree_) {
      std::filesystem::path ot_path = save_dir / ("octomap_" + timestamp + ".ot");
      if (octree_->write(ot_path.string()) && node_)
        RCLCPP_INFO(node_->get_logger(), "Saved octomap to %s", ot_path.c_str());
      else if (node_)
        RCLCPP_ERROR(node_->get_logger(), "Failed to save octomap to %s", ot_path.c_str());
    }

    const std::unordered_map<std::string, GraphNode> * nodes_ptr = nullptr;
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      if (active_graph_ && !active_graph_->nodes.empty())
        nodes_ptr = &active_graph_->nodes;
    }
    if (!nodes_ptr && !graph_nodes_.empty())
      nodes_ptr = &graph_nodes_;

    if (nodes_ptr && !nodes_ptr->empty()) {
      std::filesystem::path ply_path = save_dir / ("graph_" + timestamp + ".ply");
      std::ofstream ply(ply_path.string());
      if (ply.is_open()) {
        size_t count = nodes_ptr->size();
        ply << "ply\nformat ascii 1.0\n"
            << "element vertex " << count << "\n"
            << "property float x\nproperty float y\nproperty float z\n"
            << "property uchar red\nproperty uchar green\nproperty uchar blue\n"
            << "end_header\n";
        for (const auto & [id, gn] : *nodes_ptr) {
          int r = 255, g = 0, b = 0;
          if (gn.is_walkable && gn.is_stair_step) { r = 255; g = 255; b = 0; }
          else if (gn.is_walkable)                 { r = 0;   g = 255; b = 0; }
          ply << gn.center.x() << " " << gn.center.y() << " " << gn.center.z()
              << " " << r << " " << g << " " << b << "\n";
        }
        ply.close();
        if (node_)
          RCLCPP_INFO(node_->get_logger(),
            "Saved graph nodes (%zu) to %s", count, ply_path.c_str());
      }
    }
  } catch (const std::exception & e) {
    if (node_)
      RCLCPP_ERROR(node_->get_logger(), "Exception during shutdown save: %s", e.what());
  }
}

// =============================================================================
// Parameter change callback
// =============================================================================

rcl_interfaces::msg::SetParametersResult
OctoMappingServer::reconfigureCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto & p : parameters) {
    const std::string & pn = p.get_name();

    if (pn == name_ + ".wall_penalty_weight") {
      wall_penalty_weight_ = p.as_double(); penalties_dirty_ = true;
    } else if (pn == name_ + ".corner_penalty_weight") {
      corner_penalty_weight_ = p.as_double(); penalties_dirty_ = true;
    } else if (pn == name_ + ".centroid_penalty_weight") {
      centroid_penalty_weight_ = p.as_double(); penalties_dirty_ = true;
    } else if (pn == name_ + ".centroid_k") {
      centroid_k_ = p.as_int(); penalties_dirty_ = true;
    } else if (pn == name_ + ".centroid_lambda") {
      centroid_lambda_ = p.as_double(); penalties_dirty_ = true;
    } else if (pn == name_ + ".ransac_radius") {
      ransac_radius_ = p.as_double(); penalties_dirty_ = true;
    } else if (pn == name_ + ".penalty_spread_radius") {
      penalty_spread_radius_ = p.as_double(); penalties_dirty_ = true;
    } else if (pn == name_ + ".penalty_spread_factor") {
      penalty_spread_factor_ = p.as_double(); penalties_dirty_ = true;
    } else if (pn == name_ + ".wall_proximity_radius") {
      wall_proximity_radius_ = p.as_double(); penalties_dirty_ = true;
    } else if (pn == name_ + ".wall_proximity_height") {
      wall_proximity_height_ = p.as_double(); penalties_dirty_ = true;
    } else if (pn == name_ + ".wall_proximity_num_dirs") {
      wall_proximity_num_dirs_ = p.as_int(); penalties_dirty_ = true;
    } else if (pn == name_ + ".wall_proximity_num_z") {
      wall_proximity_num_z_ = p.as_int(); penalties_dirty_ = true;
    } else if (pn == name_ + ".walkability_recheck_passes") {
      walkability_recheck_passes_ = p.as_int(); penalties_dirty_ = true;
    } else if (pn == name_ + ".walkability_recheck_detail") {
      walkability_recheck_detail_ = p.as_int(); penalties_dirty_ = true;
    } else if (pn == name_ + ".max_step_height") {
      max_step_height_ = p.as_double(); graph_dirty_ = true; penalties_dirty_ = true;
    } else if (pn == name_ + ".stair_min_rise") {
      stair_min_rise_ = p.as_double(); graph_dirty_ = true; penalties_dirty_ = true;
    } else if (pn == name_ + ".stair_max_rise") {
      stair_max_rise_ = p.as_double(); graph_dirty_ = true; penalties_dirty_ = true;
    } else if (pn == name_ + ".enable_stair_edges") {
      enable_stair_edges_ = p.as_bool();
    } else if (pn == name_ + ".stair_adjacency_multiplier") {
      stair_adjacency_multiplier_ = p.as_double();
    } else if (pn == name_ + ".worker_thread_limit") {
      int v = p.as_int();
      if (v >= 0) worker_thread_limit_ = v;
    } else if (pn == name_ + ".enable_octomap_updates") {
      bool nv = p.as_bool();
      if (nv != enable_octomap_updates_) {
        enable_octomap_updates_ = nv;
        if (!enable_octomap_updates_) graph_dirty_ = false;
      }
    } else if (pn == name_ + ".octomap_topic") {
      std::string new_topic = p.as_string();
      if (new_topic != octomap_topic_) {
        RCLCPP_INFO(node_->get_logger(),
          "octomap_topic changed to '%s' — recreating subscription.", new_topic.c_str());
        octomap_topic_ = new_topic;
        try {
          octomap_sub_.reset();
          octomap_sub_ = node_->create_subscription<octomap_msgs::msg::Octomap>(
            octomap_topic_, 1,
            std::bind(&OctoMappingServer::octomapCallback, this, std::placeholders::_1));
        } catch (const std::exception & e) {
          RCLCPP_ERROR(node_->get_logger(),
            "Failed to recreate subscription on '%s': %s", octomap_topic_.c_str(), e.what());
        }
      }
    } else if (pn == name_ + ".incremental_graph_build") {
      bool nv = p.as_bool();
      if (nv != incremental_graph_build_) {
        incremental_graph_build_ = nv;
        if (!incremental_graph_build_) {
          processed_occupied_keys_.clear();
          nodes_needing_adjacency_update_.clear();
          graph_dirty_ = true;
        }
      }
    } else if (pn == name_ + ".publish_graph_markers") {
      bool nv = p.as_bool();
      if (nv != publish_graph_markers_) {
        publish_graph_markers_ = nv;
        if (publish_graph_markers_) {
          std::lock_guard<std::mutex> lock(graph_mutex_);
          publishGraphMarkers();
        }
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace mbf_octo_nav
