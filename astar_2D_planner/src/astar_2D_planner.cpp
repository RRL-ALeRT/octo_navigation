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
#include <astar_2D_planner/astar_2D_planner.h>

#include <queue>
#include <map>
#include <ranges>
#include <algorithm>
#include <math.h>
#include <cmath>

PLUGINLIB_EXPORT_CLASS(astar_2D_planner::Astar2DPlanner, mbf_octo_core::OctoPlanner);

namespace astar_2D_planner
{

// Structure for A* search nodes in the grid.
struct GridNode {
  std::tuple<int, int> coord;
  double f;  // f-score = g + h
  double g;  // cost from start to this node
  std::tuple<int, int> prev; //prev == coord -> null, except for (0,0)
  bool visited; // To check if neighbour is in unvisited in O(1)

  bool operator>(const GridNode& other) const {
    return f > other.f;
  }

  GridNode(std::tuple<int, int> coord, double f, double g, std::tuple<int, int> prev, bool visited) : coord(coord), f(f), g(g), prev(prev), visited(visited) {}
};

Astar2DPlanner::Astar2DPlanner()
: voxel_size_(0.1),  // double than that of octomap resolution
  z_threshold_(0.3)   // same as Z_THRESHOLD in Python
{
  // The occupancy grid will be populated by the point cloud callback.
  occupancy_grid_.clear();

  // Set a default minimum bound; this will be updated when processing point clouds.
  min_bound_ = {0.0, 0.0, 0.0};
}

Astar2DPlanner::~Astar2DPlanner() {}

bool Astar2DPlanner::isOccupied(const std::tuple<int, int, int>& pt)
{
  int x, y, z;
  std::tie(x, y, z) = pt;
  // In this example an occupied voxel is marked with the value 100.
  return occupancy_grid_[x][y][z] == 100;
}

geometry_msgs::msg::Point Astar2DPlanner::worldToGrid(const geometry_msgs::msg::Point& point)
{
  geometry_msgs::msg::Point grid_pt;
  grid_pt.x = static_cast<int>((point.x - min_bound_[0]) / voxel_size_);
  grid_pt.y = static_cast<int>((point.y - min_bound_[1]) / voxel_size_);
  grid_pt.z = static_cast<int>((point.z - min_bound_[2]) / voxel_size_);
  return grid_pt;
}

std::array<double, 3> Astar2DPlanner::gridToWorld(const std::tuple<int, int, int>& grid_pt)
{
  int x, y, z;
  std::tie(x, y, z) = grid_pt;
  double wx = x * voxel_size_ + min_bound_[0];
  double wy = y * voxel_size_ + min_bound_[1];
  double wz = z * voxel_size_ + min_bound_[2];
  return {wx, wy, wz};
}

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
  geometry_msgs::msg::Point start_grid = worldToGrid(start.pose.position);
  geometry_msgs::msg::Point goal_grid = worldToGrid(goal.pose.position);

  int grid_size_x = std::round(std::abs(goal_grid.x - start_grid.x));
  int grid_size_y = std::round(std::abs(goal_grid.y - start_grid.y));
  int grid_bottom_left_x = std::round(std::min(start_grid.x, goal_grid.x));
  int grid_bottom_left_y = std::round(std::min(start_grid.y, goal_grid.y));
  int nrGridNodes = grid_size_x*grid_size_y;
  int start_x = std::round(start_grid.x);
  int start_y = std::round(start_grid.y);
  int goal_x = std::round(goal_grid.x);
  int goal_y = std::round(goal_grid.y);

  std::vector<std::vector<GridNode>> grid;
  grid.reserve(grid_size_x);

  std::tuple<int, int> gridOrder[nrGridNodes]; //gridOrder[0] is the grid node with minimum distance from start. Rest is ordered in ascending distance.

  //Initialize grid
  for(int x = 0; x<grid_size_x; x++) {
    std::vector<GridNode> column;
    column.reserve(grid_size_y);

    for(int y = 0; y<grid_size_y; y++) {
      column.emplace_back(GridNode(std::tuple<int, int>(grid_bottom_left_x + x, grid_bottom_left_y + y), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::tuple<int, int>(grid_bottom_left_x + x, grid_bottom_left_y + y), false));
      gridOrder[y+grid_size_y*x] = std::tuple(x, y);
    }
    grid[x] = column;
  }

  //Initialize starting node
  grid[start_x][start_y].f = 0.0;
  grid[start_x][start_y].g = 0.0;
  grid[start_x][start_y].prev = std::tuple<int, int>(start_x, start_y);

  // Swap order with node (0, 0)
  std::get<0>(gridOrder[start_y+grid_size_y*start_x]) = 0;
  std::get<1>(gridOrder[start_y+grid_size_y*start_x]) = 0;
  std::get<0>(gridOrder[0]) = start_x;
  std::get<1>(gridOrder[0]) = start_y;
  
  for(int vIndex = 0; vIndex<nrGridNodes; vIndex++) {
    //TODO: If goal not reached, double grid_size_x,y and add everything outside old grid_size and add all the nodes at the edges to the queue
    
    GridNode* current = &grid[std::get<0>(gridOrder[vIndex])][std::get<0>(gridOrder[vIndex])];
    current->visited = true;
    

    if(std::get<0>(current->coord) == goal_x && std::get<1>(current->coord) == goal_y) {
      RCLCPP_ERROR(node_->get_logger(), "Goal reached. Terminating search...");
      break;
    }

    for(int x = -1; x<=1; x++) { //Get all 8 grid neighbours
      for(int y = -1; y<=1; y++) {
        if(grid[std::get<0>(current->coord)+x][std::get<1>(current->coord)+y].visited || isOccupied(std::tuple<int, int, int>(std::get<0>(current->coord)+x, std::get<1>(current->coord)+y, 0))) {
          continue; // neighbour not in unvisited or neighbour is occupied
        }

        double newF = current->g + std::sqrt(x*x + y*y); // dist(current) + cost(current, neighbour)

        if(newF < grid[std::get<0>(current->coord)+x][std::get<1>(current->coord)+y].g) { // Is dist(current) + cost(current, neighbour) < dist(neighbour)
          grid[std::get<0>(current->coord)+x][std::get<1>(current->coord)+y].g = newF; // dist(neighbour) = dist(current) + cost(current, neighbour)
          grid[std::get<0>(current->coord)+x][std::get<1>(current->coord)+y].f = newF; //TODO: Add Heuristic, if enabled

          // pred(neighbour) = current
          std::get<0>(grid[std::get<0>(current->coord)+x][std::get<1>(current->coord)+y].prev) = std::get<0>(current->coord);
          std::get<1>(grid[std::get<0>(current->coord)+x][std::get<1>(current->coord)+y].prev) = std::get<1>(current->coord);

          //Sort gridOrder, by moving the neigbour down until the next element now longer has a higher dist
          for(int i = 0; i<nrGridNodes; i++) {
            if(get<0>(grid[get<0>(gridOrder[i])][get<1>(gridOrder[i])].coord) == std::get<0>(current->coord)+x && get<1>(grid[get<0>(gridOrder[i])][get<1>(gridOrder[i])].coord) == std::get<1>(current->coord)+y) {
              while(--i > vIndex && grid[get<0>(gridOrder[i])][get<1>(gridOrder[i])].g > newF) {
                std::get<0>(grid[get<0>(gridOrder[i+1])][get<1>(gridOrder[i+1])].coord) = std::get<0>(grid[get<0>(gridOrder[i])][get<1>(gridOrder[i])].coord);
                std::get<1>(grid[get<0>(gridOrder[i+1])][get<1>(gridOrder[i+1])].coord) = std::get<1>(grid[get<0>(gridOrder[i])][get<1>(gridOrder[i])].coord);
              }
              std::get<0>(grid[get<0>(gridOrder[i+1])][get<1>(gridOrder[i+1])].coord) = std::get<0>(current->coord);
              std::get<1>(grid[get<0>(gridOrder[i+1])][get<1>(gridOrder[i+1])].coord) = std::get<1>(current->coord);
              break;
            }
          }
        }
      }
    }
  }

  //TODO: return failure case
  if(!grid[goal_x - grid_bottom_left_x][goal_y - grid_bottom_left_y].visited) {
    RCLCPP_ERROR(node_->get_logger(), "No path found using Dijkstra.");
    return mbf_msgs::action::GetPath::Result::FAILURE;
  }

  std::vector<geometry_msgs::msg::Point> grid_path;
  std::tuple<int, int> pt = grid[std::round(goal_grid.x) - grid_bottom_left_x][std::round(goal_grid.y) - grid_bottom_left_y].coord;
  while(pt != grid[start_x][start_y].coord) {
    geometry_msgs::msg::Point newP;
    newP.set__x(std::get<0>(pt));
    newP.set__y(std::get<1>(pt));
    newP.set__z(0);
    grid_path.push_back(newP);

    cost += std::sqrt(std::pow(std::get<0>(grid[std::get<0>(pt)][std::get<1>(pt)].coord) - std::get<0>(grid[std::get<0>(grid[std::get<0>(pt)][std::get<1>(pt)].prev)][std::get<1>(grid[std::get<0>(pt)][std::get<1>(pt)].prev)].coord), 2)
                    + std::pow(std::get<1>(grid[std::get<0>(pt)][std::get<1>(pt)].coord) - std::get<1>(grid[std::get<0>(grid[std::get<0>(pt)][std::get<1>(pt)].prev)][std::get<1>(grid[std::get<0>(pt)][std::get<1>(pt)].prev)].coord), 2));
    
    pt = grid[std::get<0>(pt)][std::get<1>(pt)].prev;
  }

  //Todo: fill plan with poses
  plan.clear();
  for (const auto& grid_pt : std::ranges::reverse_view(grid_path)) {
    auto world_pt = gridToWorld(std::tuple<int, int, int>(grid_pt.x, grid_pt.y, grid_pt.z));
    geometry_msgs::msg::PoseStamped pose;
    pose.header = start.header;  // Use same frame and timestamp
    pose.pose.position.x = world_pt[0];
    pose.pose.position.y = world_pt[1];
    pose.pose.position.z = world_pt[2];
    pose.pose.orientation.w = 1.0;  // Default orientation
    plan.push_back(pose);
  }

  // Publish the path for visualization.
  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = node_->now();
  path_msg.header.frame_id = "odom";
  path_msg.poses = plan;
  path_pub_->publish(path_msg);

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
  pointcloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>("/mapUGV", 1, std::bind(&Astar2DPlanner::pointcloud2Callback, this, std::placeholders::_1));

  reconfiguration_callback_handle_ = node_->add_on_set_parameters_callback(
      std::bind(&Astar2DPlanner::reconfigureCallback, this, std::placeholders::_1));

  return true;
}

void Astar2DPlanner::pointcloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Convert the ROS2 PointCloud2 message to a PCL point cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  if (cloud->empty()) {
    RCLCPP_WARN(node_->get_logger(), "No points received in the point cloud.");
    return;
  }

  // Get raw point cloud bounds
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*cloud, min_pt, max_pt);
  RCLCPP_INFO_ONCE(node_->get_logger(),
              "Raw Cloud Bounds: Min [%f, %f, %f], Max [%f, %f, %f]",
              min_pt.x, min_pt.y, min_pt.z, max_pt.x, max_pt.y, max_pt.z);

  // Store the minimum bound for coordinate conversion
  min_bound_ = {min_pt.x, min_pt.y, min_pt.z};

  // Compute grid dimensions
  int grid_size_x = static_cast<int>(std::ceil((max_pt.x - min_pt.x) / voxel_size_)) + 1;
  int grid_size_y = static_cast<int>(std::ceil((max_pt.y - min_pt.y) / voxel_size_)) + 1;
  int grid_size_z = static_cast<int>(std::ceil((max_pt.z - min_pt.z) / voxel_size_)) + 1;

  // Initialize 3D occupancy grid with -1 (unknown)
  occupancy_grid_.clear();
  occupancy_grid_.resize(grid_size_x, 
                         std::vector<std::vector<int>>(grid_size_y, 
                         std::vector<int>(grid_size_z, -1)));

  // Track occupied voxel count
  std::unordered_set<std::tuple<int, int, int>, TupleHash> occupied_voxels;

  // Populate the occupancy grid
  for (const auto& point : cloud->points) {
    int x_idx = static_cast<int>(std::round((point.x - min_pt.x) / voxel_size_));
    int y_idx = static_cast<int>(std::round((point.y - min_pt.y) / voxel_size_));
    int z_idx = static_cast<int>(std::round((point.z - min_pt.z) / voxel_size_));

    if (x_idx >= 0 && x_idx < grid_size_x &&
        y_idx >= 0 && y_idx < grid_size_y &&
        z_idx >= 0 && z_idx < grid_size_z) {
      occupancy_grid_[x_idx][y_idx][z_idx] = 100;
      occupied_voxels.insert({x_idx, y_idx, z_idx});
    }
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "Voxel Grid Size: [%d x %d x %d], Occupied Cells: %ld",
              grid_size_x, grid_size_y, grid_size_z, occupied_voxels.size());
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
