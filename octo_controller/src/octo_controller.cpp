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

#include <algorithm>
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

  // Distance and heading error to the final goal.
  double gdx = goal_pos_.pose.position.x - current_x;
  double gdy = goal_pos_.pose.position.y - current_y;
  double goal_dist = std::hypot(gdx, gdy);

  double gqx = goal_pos_.pose.orientation.x;
  double gqy = goal_pos_.pose.orientation.y;
  double gqz = goal_pos_.pose.orientation.z;
  double gqw = goal_pos_.pose.orientation.w;
  double goal_yaw = std::atan2(2.0*(gqw*gqz + gqx*gqy), 1.0 - 2.0*(gqy*gqy + gqz*gqz));
  // std::remainder gives the IEEE remainder, always in (-π, π] — correct shortest-path wrap.
  // std::fmod keeps the dividend's sign so it breaks for negative differences.
  double angle_err = std::remainder(goal_yaw - current_heading, 2.0 * M_PI);

  // Whether the goal pose carries a meaningful orientation (the A* planner emits
  // identity quaternions, so we only enforce angle alignment when it is non-identity).
  bool has_goal_orientation = std::abs(gqw - 1.0) > 1e-3 || std::abs(gqx) > 1e-3 ||
                               std::abs(gqy) > 1e-3    || std::abs(gqz) > 1e-3;

  double linear_vel = 0.0;
  double angular_vel = 0.0;
  int new_index = pursuit_index_;

  if (goal_dist < config_.arrival_fading) {
    // Parking mode: pure pursuit is disabled to prevent back-and-forth oscillation.
    // Stop linear motion; apply a P-controller for heading only if the goal has a
    // meaningful orientation, otherwise just hold still.
    linear_vel = 0.0;
    if (has_goal_orientation) {
      // Full max speed until within the fine-alignment zone, then P-controller.
      // The zone boundary (max_ang / ang_vel_factor) is where P output equals max speed,
      // so the transition is continuous — no velocity jump.
      const double fine_zone = config_.max_ang_velocity /
                               std::max(config_.ang_vel_factor, 0.1);
      if (std::abs(angle_err) > fine_zone) {
        // Lock the rotation direction on the first bang-bang tick.
        // Without this, sensor noise near ±π flips the sign of angle_err every tick,
        // causing the robot to oscillate at max speed and never converge.
        if (parking_rotation_sign_ == 0.0) {
          parking_rotation_sign_ = (angle_err >= 0.0) ? 1.0 : -1.0;
        }
        angular_vel = parking_rotation_sign_ * config_.max_ang_velocity;
      } else {
        parking_rotation_sign_ = 0.0;  // release lock; P-controller takes over
        angular_vel = config_.ang_vel_factor * angle_err;  // within [-max, max] by construction
      }
    } else {
      parking_rotation_sign_ = 0.0;
      angular_vel = 0.0;
    }
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
      "Parking: dist=%.3f angle_err=%.3f ang_vel=%.3f", goal_dist, angle_err, angular_vel);
  } else {
    std::tie(linear_vel, angular_vel, new_index) =
        purePursuit(current_x, current_y, current_heading,
                    current_plan_, pursuit_index_,
                    config_.max_lin_velocity, config_.max_search_distance);
    pursuit_index_ = new_index;
  }

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
// Direction (forward/backward) is determined automatically from the dot product of the
// vector to the lookahead point with the robot's heading: dot < 0 means the target is
// behind the robot, so we drive in reverse (negative linear velocity).
std::tuple<double, double, int> OctoController::purePursuit(
    double current_x,
    double current_y,
    double current_heading,
    const std::vector<geometry_msgs::msg::PoseStamped> & path,
    int index,
    double speed,
    double lookahead_distance)
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
        // Is the lookahead point in front of or behind the robot?
        // Positive dot product: point is in the forward half-plane -> drive forward.
        // Negative dot product: point is in the rear half-plane    -> drive backward.
        double lx = closest_point.first  - current_x;
        double ly = closest_point.second - current_y;
        bool forward = (lx * std::cos(current_heading) + ly * std::sin(current_heading)) >= 0.0;

        double target_heading;
        if (forward) {
            v = speed;
            target_heading = std::atan2(ly, lx);
        } else if (config_.backward_walking_enable) {
            // Point robot's back toward the target: aim the front in the opposite direction.
            v = -speed;
            target_heading = std::atan2(-ly, -lx);
        } else {
            // Backward walking disabled: rotate in place to face the target first.
            v = 0.0;
            target_heading = std::atan2(ly, lx);
        }
        desired_steering_angle = target_heading - current_heading;
    } else {
        // No suitable lookahead found: steer toward the last waypoint.
        double lx = path.back().pose.position.x - current_x;
        double ly = path.back().pose.position.y - current_y;
        bool forward = (lx * std::cos(current_heading) + ly * std::sin(current_heading)) >= 0.0;

        if (forward) {
            desired_steering_angle = std::atan2(ly, lx) - current_heading;
            v = speed;
        } else if (config_.backward_walking_enable) {
            desired_steering_angle = std::atan2(-ly, -lx) - current_heading;
            v = -speed;
        } else {
            // Backward walking disabled: rotate in place to face the target first.
            desired_steering_angle = std::atan2(ly, lx) - current_heading;
            v = 0.0;
        }
        index = path.size() - 1;
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
  double dx = goal_pos_.pose.position.x - current_pose_.pose.position.x;
  double dy = goal_pos_.pose.position.y - current_pose_.pose.position.y;
  double goal_distance = std::hypot(dx, dy);
  dist_tolerance  = 0.56;
  angle_tolerance = 0.06;

  // Extract yaw properly from full quaternions (orientation.z alone is NOT yaw)
  auto yaw_from_quat = [](const geometry_msgs::msg::Quaternion& q) {
    return std::atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
  };
  double current_yaw = yaw_from_quat(current_pose_.pose.orientation);
  double goal_yaw    = yaw_from_quat(goal_pos_.pose.orientation);
  double angle_err   = std::remainder(goal_yaw - current_yaw, 2.0 * M_PI);

  // Only enforce angle when the goal carries a non-identity orientation.
  // The A* planner emits identity quaternions (w=1), so angle check is skipped in that case.
  // Check all four components to match the logic in computeVelocityCommands.
  const auto& gq = goal_pos_.pose.orientation;
  bool has_goal_orientation = std::abs(gq.w - 1.0) > 1e-3 || std::abs(gq.x) > 1e-3 ||
                              std::abs(gq.y) > 1e-3        || std::abs(gq.z) > 1e-3;
  bool angle_ok = !has_goal_orientation || std::abs(angle_err) <= angle_tolerance;

  return goal_distance <= dist_tolerance && angle_ok;
}

bool OctoController::setPlan(const std::vector<geometry_msgs::msg::PoseStamped>& plan)
{
  current_plan_ = plan;
  goal_pos_ = current_plan_.back(); // Store the goal position
  cancel_requested_ = false;
  pursuit_index_ = 0;
  parking_rotation_sign_ = 0.0;
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
    } else if (parameter.get_name() == name_ + ".backward_walking_enable") {
      config_.backward_walking_enable = parameter.as_bool();
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
  { // backward_walking_enable
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "If true, the controller may drive in reverse when the lookahead point is behind the robot. "
                             "If false, the robot rotates in place to face the target first, then drives forward.";
    config_.backward_walking_enable = node->declare_parameter(name_ + ".backward_walking_enable", config_.backward_walking_enable, descriptor);
  }

  reconfiguration_callback_handle_ = node_->add_on_set_parameters_callback(std::bind(
      &OctoController::reconfigureCallback, this, std::placeholders::_1));

  return true;
}
} /* namespace octo_controller */
