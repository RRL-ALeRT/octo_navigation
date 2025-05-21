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

#include <nav_msgs/msg/occupancy_grid.hpp>  // OccupancyGrid
#include <unordered_map>                    // A*
#include <array>
#include <cmath>
#include <utility>
#include <rclcpp_action/rclcpp_action.hpp> // Action
#include <mbf_msgs/action/move_base.hpp>

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
  bool operator<(const GridNode& other) const {
    return f < other.f;
  }
};

Astar2DPlanner::Astar2DPlanner(){}
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

  // 1) check if receiving map
  if (occ_grid_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No map received yet.");
    return mbf_msgs::action::GetPath::Result::FAILURE;
  }

  // 2) world → grid
  auto [sx, sy] = worldToGrid(start.pose.position.x, start.pose.position.y);
  auto [gx, gy] = worldToGrid(goal.pose.position.x,  goal.pose.position.y);

  // Boundary check
  if (sx<0||sy<0||gx<0||gy<0||sx>=static_cast<int>(width_)||sy>=static_cast<int>(height_)||
      gx>=static_cast<int>(width_)||gy>=static_cast<int>(height_))
  {
    RCLCPP_ERROR(node_->get_logger(), "Start or goal outside map.");
    return mbf_msgs::action::GetPath::Result::FAILURE;
  }

  // 3) A*
  auto grid_path = astar({sx,sy}, {gx,gy});
  if (grid_path.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No path found using A*.");
    return mbf_msgs::action::GetPath::Result::FAILURE;
  }

  // 4) grid → world → plan
  plan.clear();
  for(const auto& p : grid_path){
    geometry_msgs::msg::PoseStamped pose;
    pose.header = start.header;
    auto w = gridToWorld(p.first, p.second);
    pose.pose.position.x = w[0];
    pose.pose.position.y = w[1];
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.push_back(pose);
  }

  cost = static_cast<double>(plan.size());

  // 5) RViz Path
  nav_msgs::msg::Path path_msg;
  path_msg.header = plan.front().header;
  path_msg.poses  = plan;
  path_pub_->publish(path_msg);

  RCLCPP_INFO_STREAM(node_->get_logger(), "Path found with length: " << cost << " steps");

  return mbf_msgs::action::GetPath::Result::SUCCESS;
}

inline std::pair<int,int> Astar2DPlanner::worldToGrid(double wx, double wy) const
{
  int gx = static_cast<int>((wx - origin_x_) / map_resolution_);
  int gy = static_cast<int>((wy - origin_y_) / map_resolution_);
  return {gx, gy};
}

inline std::array<double,2> Astar2DPlanner::gridToWorld(int gx, int gy) const
{
  double wx = gx * map_resolution_ + origin_x_;
  double wy = gy * map_resolution_ + origin_y_;
  return {wx, wy};
}

std::vector<std::pair<int,int>> Astar2DPlanner::astar(
  const std::pair<int,int>& start,
  const std::pair<int,int>& goal)
{
  struct Node {
    std::pair<int,int> xy;
    double f, g;
    bool operator>(const Node& o) const { return f > o.f; }
  };
  auto heuristic = [](const auto& a, const auto& b){
    return std::hypot(a.first - b.first, a.second - b.second);
  };

  const std::vector<std::pair<int,int>> nbr = {
    {1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}
  };

  std::priority_queue<Node,std::vector<Node>,std::greater<Node>> open;
  std::unordered_map<long long,double> gscore;
  std::unordered_map<long long,std::pair<int,int>> came;
  auto hash = [this](int x,int y){ return static_cast<long long>(y)*width_+x; };

  open.push({start, heuristic(start,goal), 0.0});
  gscore[hash(start.first,start.second)] = 0.0;

  while(!open.empty()){
    auto cur = open.top(); open.pop();
    if(cur.xy == goal){
      std::vector<std::pair<int,int>> path;
      auto node = cur.xy;
      while(came.count(hash(node.first,node.second))){
        path.push_back(node);
        node = came[hash(node.first,node.second)];
      }
      path.push_back(start);
      std::reverse(path.begin(), path.end());
      return path;
    }

    for(auto [dx,dy] : nbr){
      int nx = cur.xy.first + dx, ny = cur.xy.second + dy;
      if(nx<0||ny<0||nx>=static_cast<int>(width_)||ny>=static_cast<int>(height_))
        continue;
      if(occ_grid_[ny][nx] != 0)          // 100=occu, -1=unknown skip
        continue;

      auto tentative = std::pair<int,int>{nx, ny};
      double tentative_g = cur.g + heuristic(cur.xy, tentative);
      auto key = hash(nx,ny);
      if(!gscore.count(key) || tentative_g < gscore[key]){
        gscore[key] = tentative_g;
        came[key] = cur.xy;
        open.push({tentative,
          tentative_g + heuristic(tentative, goal),
          tentative_g});
      }
    }
  }
  return {};               // Null -> No path
}

void Astar2DPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_resolution_ = msg->info.resolution;
  origin_x_ = msg->info.origin.position.x;
  origin_y_ = msg->info.origin.position.y;
  width_  = msg->info.width;
  height_ = msg->info.height;

  occ_grid_.assign(height_, std::vector<int8_t>(width_, -1));

  // ROS OccupancyGrid data 1d append、row-major。
  for (size_t y = 0; y < height_; ++y) {
    for (size_t x = 0; x < width_; ++x) {
      occ_grid_[y][x] = msg->data[y * width_ + x];
    }
  }
}

void Astar2DPlanner::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  using namespace std::chrono_literals;

  if (!mbf_client_->wait_for_action_server(0s))
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "MoveBaseFlex action server not available!");
    return;
  }

  mbf_msgs::action::MoveBase::Goal goal;
  goal.target_pose = *msg;        // RViz Pose to MBF

  auto send_opts = rclcpp_action::Client<mbf_msgs::action::MoveBase>::SendGoalOptions();
  send_opts.goal_response_callback =
      [this](auto handle) {                                         // log after send
        if (!handle) RCLCPP_ERROR(node_->get_logger(),"Goal rejected");
        else         RCLCPP_INFO (node_->get_logger(),"Goal accepted");
      };

  mbf_client_->async_send_goal(goal, send_opts);
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

  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/mapUGV", rclcpp::QoS(1).transient_local().reliable().durability_volatile(),
    std::bind(&Astar2DPlanner::mapCallback, this, std::placeholders::_1));

  // RViz /goal_pose drive MBF -----
  goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose",        // RViz topic
    1,
    std::bind(&Astar2DPlanner::goalPoseCallback, this, std::placeholders::_1));
  mbf_client_ = rclcpp_action::create_client<mbf_msgs::action::MoveBase>(
    node_, "/move_base_flex/move_base");

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