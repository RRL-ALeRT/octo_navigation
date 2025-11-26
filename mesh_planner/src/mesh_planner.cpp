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

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>

#include <mbf_msgs/action/get_path.hpp>
#include <mesh_planner/mesh_planner.h>

#include <queue>

PLUGINLIB_EXPORT_CLASS(mesh_planner::MeshPlanner, mbf_octo_core::OctoPlanner);

namespace mesh_planner
{


MeshPlanner::MeshPlanner()
{

}
MeshPlanner::~MeshPlanner() {}

uint32_t MeshPlanner::makePlan(const geometry_msgs::msg::PoseStamped& start,
                                const geometry_msgs::msg::PoseStamped& goal,
                                double tolerance,
                                std::vector<geometry_msgs::msg::PoseStamped>& plan,
                                    double& cost,
                                    std::string& message)
{
  RCLCPP_INFO(node_->get_logger(), "Start mesh planner.");
  RCLCPP_INFO(node_->get_logger(), "Start position: x = %f, y = %f, z = %f, frame_id = %s",
              start.pose.position.x, start.pose.position.y, start.pose.position.z,
              start.header.frame_id.c_str());



  // Convert grid path back to world coordinates and build the plan.
  plan.clear();

  geometry_msgs::msg::PoseStamped pose;
  pose.header = start.header;  // Use same frame and timestamp
  pose.pose.position.x = goal.pose.position.x;
  pose.pose.position.y = goal.pose.position.y;
  pose.pose.position.z = goal.pose.position.z;
  pose.pose.orientation.w = 1.0;  // Default orientation
  plan.push_back(pose);


  // Compute a simple cost (e.g., number of steps) – replace with a proper cost calculation if desired.
  cost = static_cast<double>(plan.size());

  // Publish the path for visualization.
  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = node_->now();
  // Use the same frame as the start pose for the published path.
  path_msg.header.frame_id = start.header.frame_id;
  path_msg.poses = plan;
  path_pub_->publish(path_msg);

  RCLCPP_INFO_STREAM(node_->get_logger(), "Path found with length: " << cost << " steps");

  return mbf_msgs::action::GetPath::Result::SUCCESS;
}



void MeshPlanner::mesh_callback(const mesh_msgs::msg::MeshGeometryStamped::SharedPtr msg)
{
  // Convert the ROS2 PointCloud2 message to a PCL point cloud.
  RCLCPP_INFO(node_->get_logger(), "Mesh callback received.");
}

bool MeshPlanner::cancel()
{
  cancel_planning_ = true;
  return true;
}

bool MeshPlanner::initialize(const std::string& plugin_name, const rclcpp::Node::SharedPtr& node)
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

  // Create a subscription to the mesh topic.
  mesh_sub_ = node_->create_subscription<mesh_msgs::msg::MeshGeometryStamped>(
      "/Nav/mesh_map", 1,
      std::bind(&MeshPlanner::mesh_callback, this, std::placeholders::_1));

  // Declare planner tuning parameters (can be changed at runtime)
  z_threshold_ = node_->declare_parameter(name_ + ".z_threshold", z_threshold_);
  robot_radius_ = node_->declare_parameter(name_ + ".robot_radius", robot_radius_);
  min_vertical_clearance_ = node_->declare_parameter(name_ + ".min_vertical_clearance", min_vertical_clearance_);
  max_vertical_clearance_ = node_->declare_parameter(name_ + ".max_vertical_clearance", max_vertical_clearance_);

  RCLCPP_INFO(node_->get_logger(), "Declared parameters: %s.z_threshold=%.3f, %s.robot_radius=%.3f, %s.min_vertical_clearance=%.3f, %s.max_vertical_clearance=%.3f",
               name_.c_str(), z_threshold_, name_.c_str(), robot_radius_, name_.c_str(), min_vertical_clearance_, name_.c_str(), max_vertical_clearance_);

  reconfiguration_callback_handle_ = node_->add_on_set_parameters_callback(
      std::bind(&MeshPlanner::reconfigureCallback, this, std::placeholders::_1));

  return true;
}

rcl_interfaces::msg::SetParametersResult MeshPlanner::reconfigureCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto& parameter : parameters) {
    if (parameter.get_name() == name_ + ".cost_limit") {
      config_.cost_limit = parameter.as_double();
      RCLCPP_INFO_STREAM(node_->get_logger(), "New cost limit parameter received via dynamic reconfigure.");
    } else if (parameter.get_name() == name_ + ".z_threshold") {
      z_threshold_ = parameter.as_double();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated z_threshold to " << z_threshold_);
    } else if (parameter.get_name() == name_ + ".robot_radius") {
      robot_radius_ = parameter.as_double();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated robot_radius to " << robot_radius_);
    } else if (parameter.get_name() == name_ + ".min_vertical_clearance") {
      min_vertical_clearance_ = parameter.as_double();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated min_vertical_clearance to " << min_vertical_clearance_);
    } else if (parameter.get_name() == name_ + ".max_vertical_clearance") {
      max_vertical_clearance_ = parameter.as_double();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated max_vertical_clearance to " << max_vertical_clearance_);
    }
  }
  result.successful = true;
  return result;
}

} // namespace mesh_planner
