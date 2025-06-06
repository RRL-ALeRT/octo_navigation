/*
 *  Copyright 2025, MASCOR INSTITUTE
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
 *  author: MASCOR INSTITUTE
 *
 */

#ifndef MBF_OCTO_CORE__OCTO_CONTROLLER_H
#define MBF_OCTO_CORE__OCTO_CONTROLLER_H

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mbf_abstract_core/abstract_controller.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

namespace mbf_octo_core
{
class OctoController : public mbf_abstract_core::AbstractController
{
public:
  typedef std::shared_ptr<mbf_octo_core::OctoController> Ptr;

  OctoController() {};

  /**
   * @brief Destructor
   */
  virtual ~OctoController() {};

  /**
   * @brief Given the current position, orientation, and velocity of the robot,
   * compute velocity commands to send to the base.
   * @param pose The current pose of the robot.
   * @param velocity The current velocity of the robot.
   * @param cmd_vel Will be filled with the velocity command to be passed to the
   * robot base.
   * @param message Optional more detailed outcome as a string
   * @return Result code as described on ExePath action result (see ExePath.action)
   */
  virtual uint32_t computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                           const geometry_msgs::msg::TwistStamped& velocity,
                                           geometry_msgs::msg::TwistStamped& cmd_vel, std::string& message) = 0;

  /**
   * @brief Check if the goal pose has been achieved by the local planner
   * @param angle_tolerance The angle tolerance in which the current pose will
   * be partly accepted as reached goal
   * @param dist_tolerance The distance tolerance in which the current pose will
   * be partly accepted as reached goal
   * @return True if achieved, false otherwise
   */
  virtual bool isGoalReached(double dist_tolerance, double angle_tolerance) = 0;

  /**
   * @brief Set the plan that the local planner is following
   * @param plan The plan to pass to the local planner
   * @return True if the plan was updated successfully, false otherwise
   */
  virtual bool setPlan(const std::vector<geometry_msgs::msg::PoseStamped>& plan) = 0;

  /**
   * @brief Requests the planner to cancel, e.g. if it takes too much time.
   * @return True if a cancel has been successfully requested, false if not
   * implemented.
   */
  virtual bool cancel() = 0;

  /**
   * @brief Initializes the controller plugin with a name, a tf pointer and a mesh map pointer
   * @param plugin_name The controller plugin name, defined by the user. It defines the controller namespace
   * @param tf_ptr A shared pointer to a transformation buffer
   * @return true if the plugin has been initialized successfully
   */
  virtual bool initialize(const std::string& name,
                          const std::shared_ptr<tf2_ros::Buffer>& tf_ptr,
                          const rclcpp::Node::SharedPtr& node) = 0;
};
} /* namespace mbf_mesh_core */

#endif /* MBF_OCTO_NAVIGATION__OCTO_CONTROLLER_H */
