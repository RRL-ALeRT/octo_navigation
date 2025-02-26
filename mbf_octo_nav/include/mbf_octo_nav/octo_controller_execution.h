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

#ifndef MBF_OCTO_NAV__OCTO_CONTROLLER_EXECUTION_H
#define MBF_OCTO_NAV__OCTO_CONTROLLER_EXECUTION_H

#include <memory>

#include <mbf_abstract_nav/abstract_controller_execution.h>
#include <mbf_octo_core/octo_controller.h>


namespace mbf_octo_nav
{
/**
 * @brief The OctoControllerExecution binds a octo to the
 * AbstractControllerExecution and uses the mbf_octo_core/OctoController class
 * as base plugin interface.
 *
 * @ingroup controller_execution move_base_server
 */
class OctoControllerExecution : public mbf_abstract_nav::AbstractControllerExecution
{
public:

  /**
   * @brief Constructor
   * @param name                The user defined name of the corresponding
   * plugin
   * @param controller_ptr      The shared pointer to the plugin object
   * @param vel_pub             The velocity publisher for the controller
   * execution
   * @param goal_pub            The current goal publisher fir the controller
   * execution
   * @param tf_listener_ptr     A shared pointer to the transform listener
   * @param config              The current config object
   * @param setup_fn            A setup function called before execution
   * @param cleanup_fn          A cleanup function called after execution
   */
  OctoControllerExecution(const std::string& name, const mbf_octo_core::OctoController::Ptr& controller_ptr,
                          const mbf_utility::RobotInformation::ConstPtr& robot_info,
                          const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr& vel_pub, const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& goal_pub,
                          const rclcpp::Node::SharedPtr& node);

  /**
   * @brief Destructor
   */
  virtual ~OctoControllerExecution();

protected:
  /**
   * @brief Request plugin for a new velocity command. We override this method
   * so we can lock the local octo before calling the planner.
   * @param robot_pose         The current pose of the robot.
   * @param robot_velocity     The current velocity of the robot.
   * @param cmd_vel            Will be filled with the velocity command to be
   * passed to the robot base.
   * @param message            Optional more detailed outcome as a string.
   * @return                   Result code as described in the ExePath action
   * result and plugin's header.
   */
  virtual uint32_t computeVelocityCmd(const geometry_msgs::msg::PoseStamped& robot_pose,
                                      const geometry_msgs::msg::TwistStamped& robot_velocity,
                                      geometry_msgs::msg::TwistStamped& vel_cmd, std::string& message);

private:

  //! name of the controller plugin assigned by the class loader
  std::string controller_name_;
};

} /* namespace mbf_octo_nav */

#endif /* MBF_OCTO_NAV__OCTO_CONTROLLER_EXECUTION_H */
