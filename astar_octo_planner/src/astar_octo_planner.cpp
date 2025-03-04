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

#include <chrono>
#include <astar_octo_planner/astar_octo_planner.h>
#include <mbf_msgs/action/get_path.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

PLUGINLIB_EXPORT_CLASS(astar_octo_planner::AstarOctoPlanner, mbf_octo_core::OctoPlanner);

namespace astar_octo_planner
{

AstarOctoPlanner::AstarOctoPlanner() {}

AstarOctoPlanner::~AstarOctoPlanner() {}

uint32_t AstarOctoPlanner::makePlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal,
                            double tolerance, std::vector<geometry_msgs::msg::PoseStamped>& plan, double& cost,
                            std::string& message)
{
  //const auto& mesh = mesh_map_->mesh();
  RCLCPP_INFO(node_->get_logger(), "start astar octo planner.");

  RCLCPP_INFO(node_->get_logger(), "Start position: x = %f, y = %f, z = %f, frame_id = %s",
            start.pose.position.x, start.pose.position.y, start.pose.position.z, start.header.frame_id.c_str());

  geometry_msgs::msg::Point start_pos;
  start_pos.x = start.pose.position.x;
  start_pos.y = start.pose.position.y;
  start_pos.z = start.pose.position.z;

  geometry_msgs::msg::Point goal_pos;
  goal_pos.x = goal.pose.position.x;
  goal_pos.y = goal.pose.position.y;
  goal_pos.z = goal.pose.position.z;

  set_start_pub_->publish(start_pos);
  set_goal_pub_->publish(goal_pos);

  uint32_t outcome;

  rclcpp::Time start_time = node_->now();
  rclcpp::Duration timeout(30, 0); // 15 seconds timeout

  while (received_path_ == nullptr)
  {
    if ((node_->now() - start_time) > timeout)
    {
      RCLCPP_ERROR(node_->get_logger(), "Timeout waiting for path from DSP.");
      outcome = mbf_msgs::action::GetPath::Result::FAILURE;
      return outcome;
    }

    RCLCPP_INFO(node_->get_logger(), "Waiting for path from DSP...");
    rclcpp::sleep_for(std::chrono::milliseconds(100)); // Sleep for a short duration to avoid busy-waiting
  }

  // Path received
  RCLCPP_INFO(node_->get_logger(), "Received path from DSP.");
  for (const auto& pose : received_path_->poses)
  {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = received_path_->header;
    pose_stamped.pose = pose.pose;
    plan.push_back(pose_stamped);
  }
  received_path_ = nullptr;
  cost = 1.0; // You can calculate the actual cost based on the plan
  outcome = mbf_msgs::action::GetPath::Result::SUCCESS;

  std_msgs::msg::Header header;
  header.stamp = node_->now();
  header.frame_id = "odom";

  nav_msgs::msg::Path path_msg;
  path_msg.poses = plan;
  header.frame_id = start.header.frame_id; // Ensure the frame ID is set correctly

  path_pub_->publish(path_msg);

  RCLCPP_INFO_STREAM(node_->get_logger(), "Path length: " << cost << "m");

  return outcome;
}

bool AstarOctoPlanner::cancel()
{
  cancel_planning_ = true;
  return true;
}


void AstarOctoPlanner::dsp_path_cb(const nav_msgs::msg::Path::SharedPtr msg)
{
  received_path_ = msg;
}

bool AstarOctoPlanner::initialize(const std::string& plugin_name, const rclcpp::Node::SharedPtr& node)
{
  name_ = plugin_name;
  node_ = node;

  config_.publish_vector_field = node_->declare_parameter(name_ + ".publish_vector_field", config_.publish_vector_field);
  config_.publish_face_vectors = node_->declare_parameter(name_ + ".publish_face_vectors", config_.publish_face_vectors);
  config_.goal_dist_offset =  node->declare_parameter(name_ + ".goal_dist_offset", config_.goal_dist_offset);
  { // cost limit param
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Defines the vertex cost limit with which it can be accessed.";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 10.0;
    descriptor.floating_point_range.push_back(range);
    config_.cost_limit =  node->declare_parameter(name_ + ".cost_limit", config_.cost_limit);
  }

  path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("~/path", rclcpp::QoS(1).transient_local());

  set_start_pub_ = node_->create_publisher<geometry_msgs::msg::Point>("/dsp/set_start", rclcpp::QoS(1).transient_local());
  set_goal_pub_ = node_->create_publisher<geometry_msgs::msg::Point>("/dsp/set_goal", rclcpp::QoS(1).transient_local());

  dsp_path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
    "/dsp/path", 10, std::bind(&AstarOctoPlanner::dsp_path_cb, this, std::placeholders::_1));


  plan_path_client_ = node_->create_client<astar_octo_msgs::srv::PlanPath>("/plan_path");


  reconfiguration_callback_handle_ = node_->add_on_set_parameters_callback(std::bind(
      &AstarOctoPlanner::reconfigureCallback, this, std::placeholders::_1));

  return true;
}



rcl_interfaces::msg::SetParametersResult AstarOctoPlanner::reconfigureCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    if (parameter.get_name() == name_ + ".cost_limit") {
      config_.cost_limit = parameter.as_double();
      RCLCPP_INFO_STREAM(node_->get_logger(), "New height diff layer config through dynamic reconfigure.");
    }
  }
  result.successful = true;
  return result;
}




} /* namespace astar_octo_planner */

