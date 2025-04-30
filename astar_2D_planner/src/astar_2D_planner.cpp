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

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


#include <mbf_msgs/action/get_path.hpp>
#include <astar_2D_planner/astar_2D_planner.h>

#include <queue>

PLUGINLIB_EXPORT_CLASS(astar_2D_planner::Astar2DPlanner, mbf_octo_core::OctoPlanner);

namespace astar_2D_planner
{

// Structure for A* search nodes in the grid.
struct GridNode {
  std::tuple<int, int, int> coord;
  double f;  // f-score = g + h
  double g;  // cost from start to this node
  bool operator>(const GridNode& other) const {
    return f > other.f;
  }
};

Astar2DPlanner::Astar2DPlanner()
{

}

Astar2DPlanner::~Astar2DPlanner() {}

uint32_t Astar2DPlanner::makePlan(const geometry_msgs::msg::PoseStamped& start,
                                    const geometry_msgs::msg::PoseStamped& goal,
                                    double tolerance,
                                    std::vector<geometry_msgs::msg::PoseStamped>& plan,
                                    double& cost,
                                    std::string& message)
{
  RCLCPP_INFO(node_->get_logger(), "Start astar 2D planner.");
  RCLCPP_INFO(node_->get_logger(), "Start position: x = %f, y = %f, z = %f, frame_id = %s",
              start.pose.position.x, start.pose.position.y, start.pose.position.z,
              start.header.frame_id.c_str());

  // Convert world coordinates to grid indices using the helper function.
  //TODO: return failure case
  // if (map empty, no path found) {
  //   RCLCPP_ERROR(node_->get_logger(), "No path found using A*.");
  //   return mbf_msgs::action::GetPath::Result::FAILURE;
  // }


  //Todo: fill plan with poses
  // plan.clear();
  // for (const auto& grid_pt : grid_path) {
  //   auto world_pt = gridToWorld(grid_pt);
  //   geometry_msgs::msg::PoseStamped pose;
  //   pose.header = start.header;  // Use same frame and timestamp
  //   pose.pose.position.x = world_pt[0];
  //   pose.pose.position.y = world_pt[1];
  //   pose.pose.position.z = world_pt[2];
  //   pose.pose.orientation.w = 1.0;  // Default orientation
  //   plan.push_back(pose);
  // }

  // Compute a simple cost (e.g., number of steps) – replace with a proper cost calculation if desired.
  cost = 0.0;
  //cost = static_cast<double>(plan.size());

  // Publish the path for visualization.
  // nav_msgs::msg::Path path_msg;
  // path_msg.header.stamp = node_->now();
  // path_msg.header.frame_id = "odom";
  // path_msg.poses = plan;
  // path_pub_->publish(path_msg);

  RCLCPP_INFO_STREAM(node_->get_logger(), "Path found with length: " << cost << " steps");

  return mbf_msgs::action::GetPath::Result::SUCCESS;
}



bool Astar2DPlanner::cancel()
{
  cancel_planning_ = true;
  return true;
}

bool Astar2DPlanner::initialize(const std::string& plugin_name, const rclcpp::Node::SharedPtr& node)
{
  name_ = plugin_name;
  node_ = node;

  config_.publish_vector_field = node_->declare_parameter(name_ + ".publish_vector_field", config_.publish_vector_field);
  config_.publish_face_vectors   = node_->declare_parameter(name_ + ".publish_face_vectors", config_.publish_face_vectors);
  config_.goal_dist_offset       = node_->declare_parameter(name_ + ".goal_dist_offset", config_.goal_dist_offset);
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Defines the vertex cost limit with which it can be accessed.";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 10.0;
    descriptor.floating_point_range.push_back(range);
    config_.cost_limit = node_->declare_parameter(name_ + ".cost_limit", config_.cost_limit);
  }

  path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("~/path", rclcpp::QoS(1).transient_local());

  //ToDo: Create a subscription to the 2D cost topic.
  // pointcloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
  //     "/octomap_point_cloud_centers", 1,
  //     std::bind(&Astar2DPlanner::pointcloud2Callback, this, std::placeholders::_1));

  reconfiguration_callback_handle_ = node_->add_on_set_parameters_callback(
      std::bind(&Astar2DPlanner::reconfigureCallback, this, std::placeholders::_1));

  return true;
}

rcl_interfaces::msg::SetParametersResult Astar2DPlanner::reconfigureCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto& parameter : parameters) {
    if (parameter.get_name() == name_ + ".cost_limit") {
      config_.cost_limit = parameter.as_double();
      RCLCPP_INFO_STREAM(node_->get_logger(), "New cost limit parameter received via dynamic reconfigure.");
    }
  }
  result.successful = true;
  return result;
}

} // namespace astar_2D_planner
