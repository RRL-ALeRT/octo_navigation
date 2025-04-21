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

#include <mbf_msgs/action/exe_path.hpp>
#include <octo_controller/octo_controller.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <mbf_utility/exe_path_exception.h>

PLUGINLIB_EXPORT_CLASS(octo_controller::OctoController, mbf_octo_core::OctoController);

#define DEBUG

#ifdef DEBUG
#define DEBUG_CALL(method) method
#else
#define DEBUG_CALL(method)
#endif

namespace octo_controller
{
OctoController::OctoController()
{
}

OctoController::~OctoController()
{
}

uint32_t OctoController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                                 const geometry_msgs::msg::TwistStamped& velocity,
                                                 geometry_msgs::msg::TwistStamped& cmd_vel,
                                                 std::string& message)
{
  // Update the current pose.
  current_pose_ = pose;

  // Extract current position from the pose.
  double current_x = pose.pose.position.x;
  double current_y = pose.pose.position.y;
  
  // Compute the robot's yaw angle from the quaternion.
  double qx = pose.pose.orientation.x;
  double qy = pose.pose.orientation.y;
  double qz = pose.pose.orientation.z;
  double qw = pose.pose.orientation.w;
  double current_heading = std::atan2(2.0 * (qw * qz + qx * qy),
                                     1.0 - 2.0 * (qy * qy + qz * qz));

  // Call the pure pursuit function.
  double linear_vel = 0.0;
  double angular_vel = 0.0;
  int new_index = pursuit_index_;
  std::tie(linear_vel, angular_vel, new_index) =
      purePursuit(current_x, current_y, current_heading,
                  current_plan_, pursuit_index_,
                  config_.max_lin_velocity, config_.max_search_distance, true);

  pursuit_index_ = new_index;

  // Use the computed velocities.
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;
  cmd_vel.header.stamp = node_->now();

  if (cancel_requested_) {
    return mbf_msgs::action::ExePath::Result::CANCELED;
  }
  return mbf_msgs::action::ExePath::Result::SUCCESS;
}

// Signature: returns {v, desired_steering_angle, new_index}
std::tuple<double, double, int> OctoController::purePursuit(
    double current_x,
    double current_y,
    double current_heading,
    const std::vector<geometry_msgs::msg::PoseStamped> & path,
    int index,
    double speed,
    double lookahead_distance,
    bool forward)
{
    bool found = false;
    std::pair<double, double> closest_point;

    // Search for a point in the path that is farther than the lookahead distance.
    for (int i = index; i < static_cast<int>(path.size()); i++) {
        double x = path[i].pose.position.x;
        double y = path[i].pose.position.y;
        double distance = std::hypot(current_x - x, current_y - y);
        if (lookahead_distance < distance) {
            closest_point = {x, y};
            index = i;
            found = true;
            break;
        }
    }

    double v;
    double desired_steering_angle;
    if (found) {
        // Calculate the lookahead angle
        double lookahead_angle = std::atan2(closest_point.second - current_y,
                                            closest_point.first - current_x);
        // Compute angle difference and normalize it to [-pi, pi]
        double angle_diff = lookahead_angle - current_heading;
        angle_diff = std::fmod(angle_diff + M_PI, 2 * M_PI) - M_PI;

        // Decide the direction based on the angle difference @skpawar1305?
        // forward = (std::abs(angle_diff) < M_PI / 2);
        v = forward ? speed : -speed;

        double target_heading;
        if (forward) {
            target_heading = std::atan2(closest_point.second - current_y,
                                        closest_point.first - current_x);
        } else {
            target_heading = std::atan2(current_y - closest_point.second,
                                        current_x - closest_point.first);
        }
        desired_steering_angle = target_heading - current_heading;
    } else {
        // If no suitable point is found, use the last point in the path.
        double target_heading;
        if (forward) {
            target_heading = std::atan2(path.back().pose.position.y - current_y, path.back().pose.position.x - current_x);
        } else {
            target_heading = std::atan2(current_y - path.back().pose.position.y, current_x - path.back().pose.position.x);
        }
        desired_steering_angle = target_heading - current_heading;
        index = path.size() - 1;
        v = forward ? speed : -speed;
    }

    // Normalize desired_steering_angle to the range [-pi, pi]
    if (desired_steering_angle > M_PI) {
        desired_steering_angle -= 2 * M_PI;
    } else if (desired_steering_angle < -M_PI) {
        desired_steering_angle += 2 * M_PI;
    }

    // If the steering angle is too large, limit it and set velocity to zero.
    if (desired_steering_angle > M_PI / 6 || desired_steering_angle < -M_PI / 6) {
        int sign = (desired_steering_angle > 0) ? 1 : -1;
        desired_steering_angle = sign * (M_PI / 4);
        v = 0.0;
    }
    return std::make_tuple(v, desired_steering_angle, index);
}

bool OctoController::isGoalReached(double dist_tolerance, double angle_tolerance)
{
  // Calculate the Euclidean distance between the robot's current position and the goal position
  double dx = goal_pos_.pose.position.x - current_pose_.pose.position.x;
  double dy = goal_pos_.pose.position.y - current_pose_.pose.position.y;
  double dz = goal_pos_.pose.position.z - current_pose_.pose.position.z;
  double goal_distance = std::sqrt(dx * dx + dy * dy + dz * dz);

  // Calculate the angle difference between the robot's current orientation and the goal orientation
  double angle = std::acos(std::cos(current_pose_.pose.orientation.z - goal_pos_.pose.orientation.z));
  dist_tolerance = 0.3;
  return goal_distance <= dist_tolerance;
}

bool OctoController::setPlan(const std::vector<geometry_msgs::msg::PoseStamped>& plan)
{
  current_plan_ = plan;
  goal_pos_ = current_plan_.back(); // Store the goal position
  cancel_requested_ = false;
  pursuit_index_ = 0;
  return true;
}

bool OctoController::cancel()
{
  RCLCPP_INFO_STREAM(node_->get_logger(), "The OctoController has been requested to cancel!");
  cancel_requested_ = true;
  return true;
}

rcl_interfaces::msg::SetParametersResult OctoController::reconfigureCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    if (parameter.get_name() == name_ + ".max_lin_velocity") {
      config_.max_lin_velocity = parameter.as_double();
    } else if (parameter.get_name() == name_ + ".max_ang_velocity") {
      config_.max_ang_velocity= parameter.as_double();
    } else if (parameter.get_name() == name_ + ".arrival_fading") {
      config_.arrival_fading = parameter.as_double();
    } else if (parameter.get_name() == name_ + ".ang_vel_factor") {
      config_.ang_vel_factor = parameter.as_double();
    } else if (parameter.get_name() == name_ + ".lin_vel_factor") {
      config_.lin_vel_factor = parameter.as_double();
    } else if (parameter.get_name() == name_ + ".max_angle") {
      config_.max_angle = parameter.as_double();
    } else if (parameter.get_name() == name_ + ".max_search_radius") {
      config_.max_search_radius = parameter.as_double();
    } else if (parameter.get_name() == name_ + ".max_search_distance") {
      config_.max_search_distance = parameter.as_double();
    }
  }

  result.successful = true;
  return result;
}

bool OctoController::initialize(const std::string& plugin_name,
                                const std::shared_ptr<tf2_ros::Buffer>& tf_ptr,
                                const rclcpp::Node::SharedPtr& node)
{
  node_ = node;
  name_ = plugin_name;

  angle_pub_ = node_->create_publisher<example_interfaces::msg::Float32>("~/current_angle", rclcpp::QoS(1).transient_local());

  { // cost max_lin_velocity
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Defines the maximum linear velocity";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 5.0;
    descriptor.floating_point_range.push_back(range);
    config_.max_lin_velocity = node->declare_parameter(name_ + ".max_lin_velocity", config_.max_lin_velocity);
  }
  { // cost max_ang_velocity
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Defines the maximum angular velocity";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 2.0;
    descriptor.floating_point_range.push_back(range);
    config_.max_ang_velocity = node->declare_parameter(name_ + ".max_ang_velocity", config_.max_ang_velocity);
  }
  { // cost arrival_fading
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Distance to goal position where the robot starts to fade down the linear velocity";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 5.0;
    descriptor.floating_point_range.push_back(range);
    config_.arrival_fading = node->declare_parameter(name_ + ".arrival_fading", config_.arrival_fading);
  }
  { // cost ang_vel_factor
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Factor for angular velocity";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.1;
    range.to_value = 10.0;
    descriptor.floating_point_range.push_back(range);
    config_.ang_vel_factor = node->declare_parameter(name_ + ".ang_vel_factor", config_.ang_vel_factor);
  }
  { // cost lin_vel_factor
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Factor for linear velocity";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.1;
    range.to_value = 10.0;
    descriptor.floating_point_range.push_back(range);
    config_.lin_vel_factor = node->declare_parameter(name_ + ".lin_vel_factor", config_.lin_vel_factor);
  }
  { // cost max_angle
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "The maximum angle for the linear velocity function";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 1.0;
    range.to_value = 180.0;
    descriptor.floating_point_range.push_back(range);
    config_.max_angle = node->declare_parameter(name_ + ".max_angle", config_.max_angle);
  }
  { // cost max_search_radius
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "The maximum radius in which to search for a consecutive neighbour face";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.01;
    range.to_value = 2.0;
    descriptor.floating_point_range.push_back(range);
    config_.max_search_radius = node->declare_parameter(name_ + ".max_search_radius", config_.max_search_radius);
  }
  { // cost max_search_distance
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "The maximum distance from the surface which is accepted for projection";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.01;
    range.to_value = 2.0;
    descriptor.floating_point_range.push_back(range);
    config_.max_search_distance = node->declare_parameter(name_ + ".max_search_distance", config_.max_search_distance);
  }

  reconfiguration_callback_handle_ = node_->add_on_set_parameters_callback(std::bind(
      &OctoController::reconfigureCallback, this, std::placeholders::_1));

  return true;
}
} /* namespace octo_controller */
