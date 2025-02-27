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

  RCLCPP_INFO(node_->get_logger(), "Start position: x = %f, y = %f, z = %f",
                start.pose.position.x, start.pose.position.y, start.pose.position.z);

  uint32_t outcome;
  auto request = std::make_shared<astar_octo_msgs::srv::PlanPath::Request>();
  request->start = start;
  request->goal = goal;
  while (!plan_path_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      outcome = mbf_msgs::action::GetPath::Result::FAILURE;
    }
    RCLCPP_INFO(node_->get_logger(), "Service not available, waiting again...");
  }

    // Send the request and wait for the response
  auto future = plan_path_client_->async_send_request(request);

  auto response = future.get();
  if (!response->plan.empty()) {
    plan = response->plan;
    cost = 1.0; // You can calculate the actual cost based on the plan
    outcome = mbf_msgs::action::GetPath::Result::SUCCESS;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call service plan_path");
    outcome = mbf_msgs::action::GetPath::Result::FAILURE;
  }

  // call dijkstra with the goal pose as seed / start vertex

  //path.reverse();

  std_msgs::msg::Header header;
  header.stamp = node_->now();
  header.frame_id = "map";

  RCLCPP_INFO_STREAM(node_->get_logger(), "Path length: " << cost << "m");
  nav_msgs::msg::Path path_msg;
  path_msg.poses = plan;
  path_msg.header = header;

  path_pub_->publish(path_msg);

  RCLCPP_INFO_STREAM(node_->get_logger(), "Path length: " << cost << "m");

  return outcome;
}

bool AstarOctoPlanner::cancel()
{
  cancel_planning_ = true;
  return true;
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

