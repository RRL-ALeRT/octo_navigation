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

#include <atomic>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <octomap/OcTree.h>

#include <mbf_octo_core/octo_planner.h>
#include <mbf_octo_core/octo_mapper.h>
#include <mbf_octo_core/octo_graph_data.h>
#include <mbf_msgs/action/get_path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace astar_octo_planner
{

/**
 * @brief A* planner that operates on the walkability graph built by OctoMappingServer.
 *
 * Responsibilities (path search only):
 *  - Receive a ready graph snapshot from OctoMapper::getReadyGraph().
 *  - Snap start/goal to the nearest walkable graph node.
 *  - Run A* on the graph with node_penalty as traversal cost.
 *  - Live collision checks (vertical clearance, radial clearance, edge collision)
 *    against the current octree obtained via OctoMapper::getOctree().
 *  - Publish the resulting path.
 *
 * Everything else (octomap subscription, graph building, segmentation,
 * costmap/penalty computation) lives in OctoMappingServer.
 */
class AstarOctoPlanner : public mbf_octo_core::OctoPlanner
{
public:
  typedef std::shared_ptr<astar_octo_planner::AstarOctoPlanner> Ptr;

  AstarOctoPlanner();
  virtual ~AstarOctoPlanner();

  // ---- OctoPlanner interface -----------------------------------------------

  virtual uint32_t makePlan(const geometry_msgs::msg::PoseStamped& start,
                            const geometry_msgs::msg::PoseStamped& goal,
                            double tolerance,
                            std::vector<geometry_msgs::msg::PoseStamped>& plan,
                            double& cost,
                            std::string& message) override;

  virtual bool cancel() override;

  /**
   * @brief Initialise the planner plugin.
   * @param plugin_name  Parameter namespace prefix.
   * @param node         Shared ROS node.
   * @param mapper       Mapping server — owns the octree, graph, and costmap.
   *                     The planner reads graph/octree through this interface.
   */
  virtual bool initialize(const std::string& plugin_name,
                          const rclcpp::Node::SharedPtr& node,
                          const mbf_octo_core::OctoMapper::Ptr& mapper) override;

protected:
  /**
   * @brief Gets called whenever the node's parameters change.
   */
  rcl_interfaces::msg::SetParametersResult
  reconfigureCallback(std::vector<rclcpp::Parameter> parameters);

private:
  // ---- ROS handles ---------------------------------------------------------
  std::string name_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr body_height_path_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr reconfiguration_callback_handle_;

  // ---- Mapper (injected at initialize) -------------------------------------
  mbf_octo_core::OctoMapper::Ptr mapper_;

  // ---- Cancel flag ---------------------------------------------------------
  std::atomic_bool cancel_planning_{false};

  // ---- Planner config ------------------------------------------------------
  struct {
    bool publish_vector_field = false;
    bool publish_face_vectors = false;
    double goal_dist_offset = 0.3;
    double cost_limit = 1.0;
  } config_;

  // ---- Robot collision parameters ------------------------------------------
  double robot_radius_          = 0.1;
  double robot_width_           = 0.4;
  double robot_length_          = 1.5;
  double robot_height_          = 0.7;
  double footprint_margin_      = 0.05;
  int    footprint_samples_x_   = 3;
  int    footprint_samples_y_   = 3;
  double min_vertical_clearance_ = -0.5;
  double max_vertical_clearance_ = 0.6;

  // ---- A* live collision check parameters ----------------------------------
  int    radial_clearance_num_dirs_   = 8;
  int    radial_clearance_num_z_      = 3;
  bool   enable_radial_clearance_     = true;
  bool   enable_edge_collision_check_ = true;

  // Maximum Z distance above the query point to consider when snapping to
  // the closest graph node.  Increase for tall stairs / steep ramps.
  double max_z_above_query_ = 0.5;

  // ---- Retry lateral-offset state ------------------------------------------
  // MBF re-calls makePlan with identical arguments on each retry; we detect
  // same-goal retries here and shift the goal snap point left/right so A*
  // tries a different entry node near the goal each time.
  double retry_lateral_offset_ = 0.5;  // metres left/right of robot heading
  int    retry_count_     = 0;
  bool   has_last_goal_   = false;
  geometry_msgs::msg::PoseStamped last_goal_;

  // ---- Utility: hash for tuple<int,int,int> --------------------------------
  struct TupleHash {
    template <typename T1, typename T2, typename T3>
    std::size_t operator()(const std::tuple<T1, T2, T3>& t) const {
      auto [a, b, c] = t;
      return std::hash<T1>()(a) ^ std::hash<T2>()(b) ^ std::hash<T3>()(c);
    }
  };

  // ---- Graph helpers -------------------------------------------------------

  // Find the closest walkable graph node to world point p.
  // max_z_above_query_ limits how far above p we look (avoids snapping to
  // a stair tread far above the intended start/goal).
  std::string findClosestGraphNode(const octomap::point3d& p) const;
  std::string findClosestGraphNode(const octomap::point3d& p,
                                   const std::shared_ptr<mbf_octo_core::GraphData>& graph) const;

  // Run A* on the graph and return the ordered list of node IDs.
  std::vector<std::string> planOnGraph(
    const std::string& start_id, const std::string& goal_id,
    const std::shared_ptr<mbf_octo_core::GraphData>& graph) const;

  // ---- Live collision checks (use mapper_->getOctree()) --------------------

  // Vertical clearance above a walkable surface node.
  // Occupancy within mapper_->getMaxStepHeight() is tolerated (climbable band).
  bool hasVerticalClearance(const octomap::point3d& center, double node_size) const;

  // Radial clearance from the free space above a walkable node.
  bool hasRadialClearanceAbove(const octomap::point3d& center, double node_size) const;

  // Straight-line edge between two 3D graph nodes is free of collisions
  // above the climbable step-height band.
  bool isEdgeCollisionFree(const octomap::point3d& from, double from_size,
                           const octomap::point3d& to,   double to_size) const;

  // Temporarily clear occupied voxels in a vertical cylinder (used to remove
  // robot leg-point artifacts around the start pose before planning).
  void clearOccupiedCylinderAround(const geometry_msgs::msg::Point& center,
                                   double radius, double z_bottom, double z_top);

  // Grid/cylinder helpers (used in path post-processing / debug)
  std::array<double, 3> gridToWorld(const std::tuple<int, int, int>& grid_pt);
  bool isCylinderCollisionFree(const std::tuple<int, int, int>& coord, double radius);
};

}  // namespace astar_octo_planner

#endif  // OCTO_NAVIGATION__ASTAR_OCTO_PLANNER_H
