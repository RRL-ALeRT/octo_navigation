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

#ifndef OCTO_NAVIGATION__dijkstra_octo_planner_H
#define OCTO_NAVIGATION__DIJKSTRA_OCTO_PLANNER_H

#include <mbf_octo_core/octo_planner.h>
#include <mbf_msgs/action/get_path.hpp>
#include <nav_msgs/msg/path.hpp>

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
};

}  // namespace dijkstra_octo_planner

#endif  // OCTO_NAVIGATION__ASTAR_OCTO_PLANNER_H
