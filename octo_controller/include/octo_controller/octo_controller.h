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
 */

#ifndef OCTO_NAVIGATION__OCTO_CONTROLLER_H
#define OCTO_NAVIGATION__OCTO_CONTROLLER_H

#include <mbf_octo_core/octo_controller.h>
#include <mbf_msgs/action/get_path.hpp>
#include <example_interfaces/msg/float32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace octo_controller
{
class OctoController : public mbf_octo_core::OctoController
{
public:

  //! shared pointer typedef to simplify pointer access of the cto controller
  typedef std::shared_ptr<octo_controller::OctoController> Ptr;

  /**
   * @brief Constructor
   */
  OctoController();

  /**
   * @brief Destructor
   */
  virtual ~OctoController();

  /**
   * @brief Given the current position, orientation, and velocity of the robot,
   * compute the next velocity commands to move the robot towards the goal.
   * @param pose The current pose of the robot
   * @param velocity The current velocity of the robot
   * @param cmd_vel Computed velocity command
   * @param message Detailed outcome as string message
   * @return An mbf_msgs/ExePathResult outcome code
   */
  virtual uint32_t computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                           const geometry_msgs::msg::TwistStamped& velocity,
                                           geometry_msgs::msg::TwistStamped& cmd_vel, std::string& message) override;

  /**
   * @brief Checks if the robot reached to goal pose
   * @param pose The current pose of the robot
   * @param angle_tolerance The angle tolerance in which the current pose will
   * be partly accepted as reached goal
   * @param dist_tolerance The distance tolerance in which the current pose will
   * be partly accepted as reached goal
   * @return true if the goal is reached
   */
  virtual bool isGoalReached(double dist_tolerance, double angle_tolerance) override;

  std::tuple<double, double, int> purePursuit(
      double current_x,
      double current_y,
      double current_heading,
      const std::vector<geometry_msgs::msg::PoseStamped> & path,
      int index,
      double speed,
      double lookahead_distance,
      bool forward);

  int pursuit_index_;

  /**
   * @brief Sets the current plan to follow, it also sets the vector field
   * @param plan The plan to follow
   * @return true if the plan was set successfully, false otherwise
   */
  virtual bool setPlan(const std::vector<geometry_msgs::msg::PoseStamped>& plan) override;

  /**
   * @brief Requests the planner to cancel, e.g. if it takes too much time
   * @return True if cancel has been successfully requested, false otherwise
   */
  virtual bool cancel() override;

  /**
   * @brief Initializes the controller plugin with a name, a tf pointer and a octo map pointer
   * @param plugin_name The controller plugin name, defined by the user. It defines the controller namespace
   * @param tf_ptr A shared pointer to a transformation buffer
   * @return true if the plugin has been initialized successfully
   */
  virtual bool initialize(const std::string& plugin_name,
                          const std::shared_ptr<tf2_ros::Buffer>& tf_ptr,
                          const rclcpp::Node::SharedPtr& node) override;

  /**
   * @brief reconfigure callback function which is called if a dynamic reconfiguration were triggered.
   */
  rcl_interfaces::msg::SetParametersResult reconfigureCallback(std::vector<rclcpp::Parameter> parameters);

private:
  //! shared pointer to node in which this plugin runs
  rclcpp::Node::SharedPtr node_;

  //! the user defined plugin name
  std::string name_;

  //! the current set plan
  std::vector<geometry_msgs::msg::PoseStamped> current_plan_;

  //! the goal and robot pose
  //mesh_map::Vector goal_pos_, robot_pos_;
  geometry_msgs::msg::PoseStamped goal_pos_, current_pose_;
  //! the goal's and robot's orientation
  //mesh_map::Normal goal_dir_, robot_dir_;


  //! publishes the angle between the robots orientation and the goal vector field for debug purposes
  rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr angle_pub_;

  //! flag to handle cancel requests
  std::atomic_bool cancel_requested_;

  // handle of callback for changing parameters dynamically
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr reconfiguration_callback_handle_;

  geometry_msgs::msg::Twist::SharedPtr received_twist_;

  struct {
    double max_lin_velocity = 1.0;
    double max_ang_velocity = 0.5;
    double arrival_fading = 0.5;
    double ang_vel_factor = 1.0;
    double lin_vel_factor = 1.0;
    double max_angle = 20.0;
    double max_search_radius = 0.4;
    double max_search_distance = 0.4;
  } config_;
};

} /* namespace octo_controller */
#endif /* OCTO_NAVIGATION__OCTO_CONTROLLER_H */
