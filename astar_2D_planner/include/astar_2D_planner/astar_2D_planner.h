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

#ifndef OCTO_NAVIGATION__ASTAR_2D_PLANNER_H
#define OCTO_NAVIGATION__ASTAR_2D_PLANNER_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <mbf_octo_core/octo_planner.h>
#include <mbf_msgs/action/get_path.hpp>

namespace astar_2D_planner
{

class Astar2DPlanner : public mbf_octo_core::OctoPlanner
{
public:
  typedef std::shared_ptr<astar_2D_planner::Astar2DPlanner> Ptr;

  Astar2DPlanner();

  /**
   * @brief Destructor
   */
  virtual ~Astar2DPlanner();

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
  // Utility functions of the 3D Planner.
  // // Callback for point cloud subscription.
  // void pointcloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
 // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
   // Occupancy grid represented as a 3D vector.
  // std::vector<std::vector<std::vector<int>>> occupancy_grid_;
  // geometry_msgs::msg::Point find_nearest_3d_point(geometry_msgs::msg::Point point, const std::vector<std::vector<std::vector<int>>>& array_3d);
  // geometry_msgs::msg::Point worldToGrid(const geometry_msgs::msg::Point & point);
  // std::array<double, 3> gridToWorld(const std::tuple<int, int, int>& grid_pt);
  // bool isWithinBounds(const std::tuple<int, int, int>& pt);
  // bool isWithinBounds(const geometry_msgs::msg::Point & pt);
  // bool isOccupied(const std::tuple<int, int, int>& pt);
  // bool hasNoOccupiedCellsAbove(const std::tuple<int, int, int>& coord,
  //                                               double vertical_min, double vertical_range);
  // bool isCylinderCollisionFree(const std::tuple<int, int, int>& coord, double radius);
  // std::vector<std::tuple<int, int, int>> astar(const std::tuple<int, int, int>& start,
  //                                              const std::tuple<int, int, int>& goal);
  // Voxel grid parameters.
  // double voxel_size_;
  // double z_threshold_;
  // double robot_radius_ = 0.35;
  // double min_vertical_clearance_ = 0.4;
  // double max_vertical_clearance_ = 0.6;

  // // Minimum bound for the occupancy grid.
  // std::array<double, 3> min_bound_;

  // // Hash function for tuple<int, int, int> to track unique occupied voxels
  // struct TupleHash {
  //   template <typename T1, typename T2, typename T3>
  //   std::size_t operator()(const std::tuple<T1, T2, T3>& t) const {
  //     auto [a, b, c] = t;
  //     return std::hash<T1>()(a) ^ std::hash<T2>()(b) ^ std::hash<T3>()(c);
  //   }
  // };
};

}  // namespace astar_2D_planner

#endif  // OCTO_NAVIGATION__ASTAR_2D_PLANNER_H
