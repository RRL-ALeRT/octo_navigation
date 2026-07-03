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

#ifndef MBF_OCTO_NAV__OCTO_MAPPING_SERVER_H
#define MBF_OCTO_NAV__OCTO_MAPPING_SERVER_H

#include <atomic>
#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <mbf_octo_core/octo_mapper.h>
#include <mbf_octo_core/octo_graph_data.h>

namespace mbf_octo_nav
{

/**
 * @brief Owns the live octomap, builds the walkability graph, and computes the
 *        penalty costmap.  Implements mbf_octo_core::OctoMapper so it can be
 *        injected into any OctoPlanner plugin.
 *
 * Responsibilities:
 *  - Subscribe to the octomap topic and merge/replace the octree.
 *  - Maintain a double-buffered GraphData (active_graph_) that grows
 *    incrementally as new scans arrive.
 *  - Classify voxels as floor / wall / stair using multi-threaded checks.
 *  - Compute the penalty costmap (centroid-shift + graph-border) on demand.
 *  - Expose the ready graph to planner plugins via getReadyGraph().
 */
class OctoMappingServer : public mbf_octo_core::OctoMapper
{
public:
  using Ptr = std::shared_ptr<OctoMappingServer>;

  OctoMappingServer();
  virtual ~OctoMappingServer();

  /**
   * @brief Initialise the server: declare ROS params, subscribe to octomap,
   *        start the background graph-build timer.
   * @param name  Parameter namespace prefix (e.g. "octo_mapping_server").
   * @param node  Shared ROS node.
   */
  void initialize(const std::string & name, const rclcpp::Node::SharedPtr & node);

  // ---- OctoMapper interface ------------------------------------------------

  std::shared_ptr<mbf_octo_core::GraphData> getReadyGraph() override;
  void markPenaltiesDirty() override;
  void publishAdditionalMarkers(
    const visualization_msgs::msg::MarkerArray & ma) override;

  const octomap::OcTree * getOctree() const override;
  double getVoxelSize() const override;
  std::string getMapFrame() const override;
  void getBounds(std::array<double, 3> & min_b,
                 std::array<double, 3> & max_b) const override;
  double getMaxStepHeight() const override;

private:
  // ---- ROS handles ---------------------------------------------------------
  rclcpp::Node::SharedPtr node_;
  std::string name_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::TimerBase::SharedPtr graph_build_timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr graph_marker_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr reconfigure_handle_;

  // ---- Octree --------------------------------------------------------------
  std::unique_ptr<octomap::OcTree> octree_;
  double active_voxel_size_ = 0.1;
  std::string map_frame_;

  // ---- Spatial bounds (never-shrink) ---------------------------------------
  std::array<double, 3> min_bound_{0.0, 0.0, 0.0};
  std::array<double, 3> max_bound_{0.0, 0.0, 0.0};
  int grid_size_x_ = 0;
  int grid_size_y_ = 0;
  int grid_size_z_ = 0;

  // ---- Double-buffered graph -----------------------------------------------
  std::shared_ptr<mbf_octo_core::GraphData> active_graph_;
  std::mutex graph_mutex_;
  std::atomic_bool graph_dirty_{false};
  std::atomic_bool graph_building_{false};
  std::atomic_bool penalties_dirty_{true};

  // ---- Legacy temp-storage members (used as scratch by build functions) ----
  // These mirror GraphData fields but are needed because buildConnectivityGraph()
  // writes to them and buildConnectivityGraphInto() copies from them.
  std::unordered_map<std::string, mbf_octo_core::GraphNode> graph_nodes_;
  std::unordered_map<std::string, std::vector<std::string>> graph_adj_;
  std::unordered_set<std::string> processed_occupied_keys_;
  std::unordered_set<std::string> nodes_needing_adjacency_update_;

  // ---- Shutdown save guard -------------------------------------------------
  std::atomic_bool maps_saved_{false};

  // ---- Octomap sensor model ------------------------------------------------
  double octomap_prob_hit_   = 0.7;
  double octomap_prob_miss_  = 0.3;
  double octomap_thres_      = 0.5;
  double octomap_clamp_min_  = 0.1;
  double octomap_clamp_max_  = 0.95;

  // ---- Subscription / build control ----------------------------------------
  std::string octomap_topic_        = "/octomap_binary_local";
  bool enable_octomap_updates_      = true;
  bool incremental_graph_build_     = true;

  // ---- Graph build parameters ----------------------------------------------
  double voxel_size_                = 0.1;
  double z_threshold_               = 0.3;
  double max_surface_distance_      = 0.25;
  double max_step_height_           = 0.30;

  // Floor support
  double min_floor_support_ratio_   = 0.4;
  int    floor_support_num_dirs_    = 8;

  // Wall proximity classification (independent of robot_height_)
  double wall_proximity_height_     = 0.7;
  double wall_proximity_radius_     = 0.5;
  int    wall_proximity_num_dirs_   = 16;
  int    wall_proximity_num_z_      = 3;

  // Walkability revalidation (called inside getReadyGraph)
  int    walkability_recheck_passes_  = 1;
  int    walkability_recheck_detail_  = 1;

  // Stair detection / augmentation
  bool   enable_stair_edges_          = true;
  double stair_xy_radius_             = 0.15;
  double stair_vertical_margin_       = 0.05;
  double stair_adjacency_multiplier_  = 5.0;
  double max_bridge_dist_             = 0.3;
  double stair_min_rise_              = 0.08;
  double stair_max_rise_              = 0.35;
  double stair_max_xy_dist_           = 0.40;
  int    stair_min_chain_length_      = 3;
  double stair_max_tread_thickness_   = 0.25;

  // Robot min clearance (used by isStairStep internal check)
  double min_vertical_clearance_      = -0.5;

  // Hole filling (cage raycast over the collapsed 2D column grid)
  bool   enable_hole_filling_         = true;
  bool   hole_fill_on_incremental_    = false;  // also fill during incremental updates
  int    hole_fill_max_area_          = 0;      // max hole size in cells, 0 = unlimited
  double hole_fill_min_border_ratio_  = 0.5;    // min fraction of walkable border columns

  // ---- Penalty / costmap parameters ----------------------------------------
  double wall_penalty_weight_         = 2.0;
  double corner_radius_               = 0.20;
  double corner_penalty_weight_       = 5.0;
  double corner_angle_thresh_deg_     = 45.0;

  // Sector-histogram detector
  int    sector_bins_                 = 8;
  double sector_radius_               = 0.40;
  double sector_peak_thresh_          = 0.55;
  int    sector_min_inliers_          = 4;

  // Centroid-shift detector
  int    centroid_k_                  = 12;
  double centroid_lambda_             = 1.0;
  double centroid_penalty_weight_     = 2.0;

  // RANSAC (declared for config parity, not actively used in penalty computation)
  double ransac_radius_               = 2.0;
  double ransac_dist_thresh_          = 0.1;
  int    ransac_min_inliers_          = 16;
  int    ransac_iterations_           = 30;

  // Penalty spread
  double penalty_spread_radius_       = 0.5;
  double penalty_spread_factor_       = 1.0;

  // Threading
  int    worker_thread_limit_         = 10;

  // ---- Visualization -------------------------------------------------------
  bool publish_graph_markers_         = true;

  // ---- Private methods: octomap callback & bounds --------------------------
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);

  // ---- Private methods: graph building -------------------------------------
  void buildConnectivityGraph(double eps = 0.05);
  void buildConnectivityGraphInto(std::shared_ptr<mbf_octo_core::GraphData> & target,
                                   double eps = 0.05);
  void updateConnectivityGraphIncremental(double eps = 0.05);
  void updateConnectivityGraphIncrementalInto(std::shared_ptr<mbf_octo_core::GraphData> & target,
                                               double eps = 0.05);

  // ---- Private methods: voxel classification -------------------------------
  bool hasFloorSupport(double x, double y, double z, double node_size) const;
  bool isStairStep(double x, double y, double voxel_top, double node_size) const;
  bool hasVerticalClearance(const octomap::point3d & center, double node_size,
                             double check_height) const;
  void detectAndAugmentStairs();

  // ---- Private methods: hole filling ----------------------------------------
  /**
   * @brief Classify enclosed gaps in the collapsed 2D column grid via a cage of
   *        border raycasts (splitting along walls on every hit), then fill each
   *        interior hole with synthetic walkable nodes at the z-level of its
   *        walkable border columns.
   * @return Number of synthetic nodes created (also queued for adjacency update).
   */
  size_t fillMapHoles();

  // ---- Private methods: walkability revalidation ---------------------------
  size_t revalidateWalkability(std::shared_ptr<mbf_octo_core::GraphData> & graph,
                                int max_passes, int detail_level);

  // ---- Private methods: penalty / costmap computation ---------------------
  void computePendingPenalties(std::shared_ptr<mbf_octo_core::GraphData> & graph);

  // ---- Private methods: visualization --------------------------------------
  void publishGraphMarkers();
  void publishGraphMarkers(const std::shared_ptr<mbf_octo_core::GraphData> & graph);

  // ---- Private methods: persistence ----------------------------------------
  void saveMapsOnShutdown();

  // ---- Parameter change callback ------------------------------------------
  rcl_interfaces::msg::SetParametersResult
  reconfigureCallback(std::vector<rclcpp::Parameter> parameters);
};

}  // namespace mbf_octo_nav

#endif  // MBF_OCTO_NAV__OCTO_MAPPING_SERVER_H
