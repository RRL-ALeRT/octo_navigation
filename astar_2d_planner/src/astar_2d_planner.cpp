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
#include <astar_2d_planner/astar_2d_planner.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <queue>

#define SmoothPath 1

PLUGINLIB_EXPORT_CLASS(astar_2d_planner::Astar2dPlanner, mbf_octo_core::OctoPlanner);

namespace astar_2d_planner
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

Astar2dPlanner::~Astar2dPlanner() {}
Astar2dPlanner::Astar2dPlanner() {}

//=========================main=============================
//when move_base_flex receive the goal it will start makePlan function
uint32_t Astar2dPlanner::makePlan(const geometry_msgs::msg::PoseStamped& start,
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
  
  // 5) Poblish non smooth first
  nav_msgs::msg::Path path_msg;
  path_msg.header = plan.front().header;
  path_msg.poses  = plan;
  path_pub_->publish(path_msg);
  cost = static_cast<double>(plan.size());
  RCLCPP_INFO_STREAM(node_->get_logger(), "Path found with length: " << cost << " steps");
  
  // 6) Do smooth path
  if(SmoothPath)
  {
    auto smooth_grid = pruneAndShortcut(grid_path);
    plan.clear();
    for(const auto& p : smooth_grid){
      geometry_msgs::msg::PoseStamped pose;
      pose.header = start.header;
      auto w = gridToWorld(p.first, p.second);
      pose.pose.position.x = w[0];
      pose.pose.position.y = w[1];
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      plan.push_back(pose);
    }
  }
  // 6-1) Do smooth
  if(SmoothPath)
  {
    std::vector<std::array<double,2>> world_pts;
    for(auto& ps : plan)
      world_pts.push_back({ps.pose.position.x, ps.pose.position.y});

    auto spline_pts = catmullRom(world_pts, 0.1);   // dot/0.1 m
    plan.clear();
    for(auto& w : spline_pts){
      geometry_msgs::msg::PoseStamped ps;
      ps.header = start.header;
      ps.pose.position.x = w[0];
      ps.pose.position.y = w[1];
      ps.pose.orientation.w = 1.0;
      plan.push_back(ps);
    }
  }
  
  // 6-3) Publish smooth Path
  if(SmoothPath)
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header = plan.front().header;
    path_msg.poses  = plan;
    path_pub_smooth_->publish(path_msg);
    cost = static_cast<double>(plan.size());
    RCLCPP_INFO_STREAM(node_->get_logger(), "Smooth Path found with length: " << cost << " steps");
  }

  return mbf_msgs::action::GetPath::Result::SUCCESS;
}

//=============testing function for smooth path=================

inline bool Astar2dPlanner::isOccupied(int gx,int gy) const {
  return occ_grid_[gy][gx] == 100;
}

inline bool Astar2dPlanner::inBounds(int x, int y) const {
  return x >= 0 && y >= 0 &&
         x < static_cast<int>(width_) &&
         y < static_cast<int>(height_);
}

bool Astar2dPlanner::isLineFree(int x0,int y0,int x1,int y1) const
{
  if(!inBounds(x0,y0) || !inBounds(x1,y1))
    return false;                       // ← check

  int dx = std::abs(x1 - x0),  sx = x0 < x1 ? 1 : -1;
  int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy; // Bresenham
  while(true){
    if (isOccupied(x0,y0)) return false;
    if (x0==x1 && y0==y1) break;
    int e2 = 2*err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
  return true;
}

std::vector<std::pair<int,int>>
Astar2dPlanner::pruneAndShortcut(const std::vector<std::pair<int,int>>& in) const
{
  if(in.size() <= 3) return in;
  // --- ① collinearity prune ---
  std::vector<std::pair<int,int>> pts;
  pts.push_back(in.front());
  for(size_t i=1;i+1<in.size();++i){
    auto a=pts.back(), b=in[i], c=in[i+1];
    int vx1=b.first-a.first,  vy1=b.second-a.second;
    int vx2=c.first-b.first,  vy2=c.second-b.second;
    if(vx1*vy2 - vy1*vx2 != 0)   // Preserve Cornor
      pts.push_back(b);
  }
  pts.push_back(in.back());

  // --- ② line‑of‑sight shortcut ---
  std::vector<std::pair<int,int>> out;
  size_t i = 0;
  while (i < pts.size() - 1)
  {
    size_t far = i + 1;
    // Far to near, if obsticle then break
    for (size_t j = pts.size() - 1; j > i + 1; --j)
    {
      if (isLineFree(pts[i].first,  pts[i].second,
                     pts[j].first,  pts[j].second))
      {
        far = j;
        break;
      }
    }
    out.emplace_back(pts[i]);
    i = far;                      // jump to farest
  }
  out.emplace_back(pts.back());
  // while(i<pts.size()){
  //   out.push_back(pts[i]);
  //   size_t j=pts.size()-1;
  //   for(; j>i+1; --j){
  //     if(isLineFree(pts[i].first,pts[i].second,
  //                   pts[j].first,pts[j].second))
  //       break;
  //   }
  //   i = j;
  // }
  return out;
}

std::vector<std::array<double,2>>
Astar2dPlanner::catmullRom(const std::vector<std::array<double,2>>& ctrl,
                           double ds /* 取樣間隔 */) const
{
  if(ctrl.size()<4) return ctrl;
  std::vector<std::array<double,2>> samp;
  for(size_t i=1;i+2<ctrl.size();++i){
    auto p0=ctrl[i-1], p1=ctrl[i], p2=ctrl[i+1], p3=ctrl[i+2];
    for(double t=0; t<1.0; t+=0.05){        // 0.05 → 每段 20 個點
      double t2=t*t, t3=t2*t;
      double x = 0.5*((2*p1[0]) + (-p0[0]+p2[0])*t + (2*p0[0]-5*p1[0]+4*p2[0]-p3[0])*t2 + (-p0[0]+3*p1[0]-3*p2[0]+p3[0])*t3);
      double y = 0.5*((2*p1[1]) + (-p0[1]+p2[1])*t + (2*p0[1]-5*p1[1]+4*p2[1]-p3[1])*t2 + (-p0[1]+3*p1[1]-3*p2[1]+p3[1])*t3);
      if(samp.empty() || std::hypot(x-samp.back()[0],y-samp.back()[1]) > ds)
        samp.push_back({x,y});
    }
  }
  samp.push_back(ctrl.back());
  return samp;
}

//=========================function=============================

inline std::pair<int,int> Astar2dPlanner::worldToGrid(double wx, double wy) const
{
  int gx = static_cast<int>((wx - origin_x_) / map_resolution_);
  int gy = static_cast<int>((wy - origin_y_) / map_resolution_);
  return {gx, gy};
}

inline std::array<double,2> Astar2dPlanner::gridToWorld(int gx, int gy) const
{
  double wx = gx * map_resolution_ + origin_x_;
  double wy = gy * map_resolution_ + origin_y_;
  return {wx, wy};
}

std::vector<std::pair<int,int>> Astar2dPlanner::astar(
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

geometry_msgs::msg::PoseStamped Astar2dPlanner::getCurrentPose(
  const std::string& target_frame, 
  const std::string& source_frame)
{
  geometry_msgs::msg::PoseStamped pose;
  try
  {
    // rclcpp::Time(0) means latest available transform
    auto transform = tf_buffer_->lookupTransform(target_frame, source_frame, rclcpp::Time(0));
    
    // Log the transform info
    RCLCPP_INFO(node_->get_logger(), "Got transform from '%s' to '%s' at time %u.%u",
      target_frame.c_str(), source_frame.c_str(),
      transform.header.stamp.sec, transform.header.stamp.nanosec);

    RCLCPP_INFO(node_->get_logger(),
      "Translation: x=%.3f, y=%.3f, z=%.3f",
      transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z);

    RCLCPP_INFO(node_->get_logger(),
      "Rotation: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
      transform.transform.rotation.x,
      transform.transform.rotation.y,
      transform.transform.rotation.z,
      transform.transform.rotation.w);

    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_ERROR(node_->get_logger(), "TF Error: %s", ex.what());
    pose.header.stamp = node_->get_clock()->now(); // fallback to current time
    pose.header.frame_id = target_frame;
    pose.pose.orientation.w = 1.0; // identity quaternion
    pose.pose.position.x = pose.pose.position.y = pose.pose.position.z = 0.0;
  }
  return pose;
}


//=========================callback=============================

void Astar2dPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
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

bool Astar2dPlanner::cancel()
{
  cancel_planning_ = true;
  return true;
}

//=========================initialize============================
bool Astar2dPlanner::initialize(const std::string& plugin_name, const rclcpp::Node::SharedPtr& node)
{
  name_ = plugin_name;
  node_ = node;
  
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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
  path_pub_smooth_ = node_->create_publisher<nav_msgs::msg::Path>("~/path_smooth", rclcpp::QoS(1).transient_local());

  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/mapUGV", rclcpp::QoS(1).transient_local().reliable().durability_volatile(),
    std::bind(&Astar2dPlanner::mapCallback, this, std::placeholders::_1));

  reconfiguration_callback_handle_ = node_->add_on_set_parameters_callback(
      std::bind(&Astar2dPlanner::reconfigureCallback, this, std::placeholders::_1));

  return true;
}

rcl_interfaces::msg::SetParametersResult Astar2dPlanner::reconfigureCallback(std::vector<rclcpp::Parameter> parameters)
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

} // namespace astar_2d_planner