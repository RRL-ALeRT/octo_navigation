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
#include <astar_octo_planner/astar_octo_planner.h>

#include <queue>

PLUGINLIB_EXPORT_CLASS(astar_octo_planner::AstarOctoPlanner, mbf_octo_core::OctoPlanner);

namespace astar_octo_planner
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

AstarOctoPlanner::AstarOctoPlanner()
: voxel_size_(0.1),  // double than that of octomap resolution
  z_threshold_(0.3)   // same as Z_THRESHOLD in Python
{
  // The occupancy grid will be populated by the point cloud callback.
  occupancy_grid_.clear();

  // Set a default minimum bound; this will be updated when processing point clouds.
  min_bound_ = {0.0, 0.0, 0.0};
}

AstarOctoPlanner::~AstarOctoPlanner() {}

uint32_t AstarOctoPlanner::makePlan(const geometry_msgs::msg::PoseStamped& start,
                                    const geometry_msgs::msg::PoseStamped& goal,
                                    double tolerance,
                                    std::vector<geometry_msgs::msg::PoseStamped>& plan,
                                    double& cost,
                                    std::string& message)
{
  RCLCPP_INFO(node_->get_logger(), "Start astar octo planner.");
  RCLCPP_INFO(node_->get_logger(), "Start position: x = %f, y = %f, z = %f, frame_id = %s",
              start.pose.position.x, start.pose.position.y, start.pose.position.z,
              start.header.frame_id.c_str());

  // Convert world coordinates to grid indices using the helper function.
  auto start_grid_pt = worldToGrid(start.pose.position);
  auto goal_grid_pt = worldToGrid(goal.pose.position);
  start_grid_pt = find_nearest_3d_point(start_grid_pt, occupancy_grid_);
  goal_grid_pt = find_nearest_3d_point(goal_grid_pt, occupancy_grid_);

  if (!isWithinBounds(start_grid_pt)) {
    RCLCPP_WARN(node_->get_logger(), "Start grid coordinates are out of bounds.");
    return mbf_msgs::action::GetPath::Result::FAILURE;
  }

  if (!isWithinBounds(goal_grid_pt)) {
    RCLCPP_WARN(node_->get_logger(), "Goal grid coordinates are out of bounds.");
    return mbf_msgs::action::GetPath::Result::FAILURE;
  }

  // Run A* search on the occupancy grid.
  auto grid_path = astar(
    std::make_tuple(start_grid_pt.x, start_grid_pt.y, start_grid_pt.z),
    std::make_tuple(goal_grid_pt.x, goal_grid_pt.y, goal_grid_pt.z)
  );
  if (grid_path.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No path found using A*.");
    return mbf_msgs::action::GetPath::Result::FAILURE;
  }

  // Convert grid path back to world coordinates and build the plan.
  plan.clear();
  for (const auto& grid_pt : grid_path) {
    auto world_pt = gridToWorld(grid_pt);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = start.header;  // Use same frame and timestamp
    pose.pose.position.x = world_pt[0];
    pose.pose.position.y = world_pt[1];
    pose.pose.position.z = world_pt[2];
    pose.pose.orientation.w = 1.0;  // Default orientation
    plan.push_back(pose);
  }

  // Compute a simple cost (e.g., number of steps) – replace with a proper cost calculation if desired.
  cost = static_cast<double>(plan.size());

  // Publish the path for visualization.
  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = node_->now();
  path_msg.header.frame_id = "odom";
  path_msg.poses = plan;
  path_pub_->publish(path_msg);

  RCLCPP_INFO_STREAM(node_->get_logger(), "Path found with length: " << cost << " steps");

  return mbf_msgs::action::GetPath::Result::SUCCESS;
}

geometry_msgs::msg::Point AstarOctoPlanner::find_nearest_3d_point(geometry_msgs::msg::Point point, const std::vector<std::vector<std::vector<int>>>& array_3d) {
  int nearest_x = -1, nearest_y = -1, nearest_z = -1;
  double min_distance = std::numeric_limits<double>::max();

  for (size_t i = 0; i < array_3d.size(); ++i) {
    for (size_t j = 0; j < array_3d[i].size(); ++j) {
      for (size_t k = 0; k < array_3d[i][j].size(); ++k) {
        if (array_3d[i][j][k] == 100) {
          double distance = std::sqrt(
            std::pow(static_cast<double>(i - point.x), 2) +
            std::pow(static_cast<double>(j - point.y), 2) +
            std::pow(static_cast<double>(k - point.z), 2)
          );

          if (distance < min_distance) {
            min_distance = distance;
            nearest_x = static_cast<int>(i);
            nearest_y = static_cast<int>(j);
            nearest_z = static_cast<int>(k);
          }
        }
      }
    }
  }

  geometry_msgs::msg::Point nearest;
  nearest.x = nearest_x;
  nearest.y = nearest_y;
  nearest.z = nearest_z;

  return nearest;
}

geometry_msgs::msg::Point AstarOctoPlanner::worldToGrid(const geometry_msgs::msg::Point& point)
{
  geometry_msgs::msg::Point grid_pt;
  grid_pt.x = static_cast<int>((point.x - min_bound_[0]) / voxel_size_);
  grid_pt.y = static_cast<int>((point.y - min_bound_[1]) / voxel_size_);
  grid_pt.z = static_cast<int>((point.z - min_bound_[2]) / voxel_size_);
  return grid_pt;
}

std::array<double, 3> AstarOctoPlanner::gridToWorld(const std::tuple<int, int, int>& grid_pt)
{
  int x, y, z;
  std::tie(x, y, z) = grid_pt;
  double wx = x * voxel_size_ + min_bound_[0];
  double wy = y * voxel_size_ + min_bound_[1];
  double wz = z * voxel_size_ + min_bound_[2];
  return {wx, wy, wz};
}

bool AstarOctoPlanner::isWithinBounds(const std::tuple<int, int, int>& pt)
{
  int x, y, z;
  std::tie(x, y, z) = pt;
  if (x < 0 || y < 0 || z < 0)
    return false;
  if (x >= static_cast<int>(occupancy_grid_.size()))
    return false;
  if (y >= static_cast<int>(occupancy_grid_[0].size()))
    return false;
  if (z >= static_cast<int>(occupancy_grid_[0][0].size()))
    return false;
  return true;
}

bool AstarOctoPlanner::isWithinBounds(const geometry_msgs::msg::Point& pt)
{
  return isWithinBounds(std::make_tuple(pt.x, pt.y, pt.z));
}

bool AstarOctoPlanner::isOccupied(const std::tuple<int, int, int>& pt)
{
  int x, y, z;
  std::tie(x, y, z) = pt;
  // In this example an occupied voxel is marked with the value 100.
  return occupancy_grid_[x][y][z] == 100;
}

bool AstarOctoPlanner::hasNoOccupiedCellsAbove(const std::tuple<int, int, int>& coord, 
                                               double vertical_min, double vertical_range)
{
  int x, y, z;
  std::tie(x, y, z) = coord;

  int z_min = z + static_cast<int>(vertical_min / voxel_size_);
  int z_max = z + static_cast<int>(vertical_range / voxel_size_);

  for (int z_check = z_min; z_check <= z_max; ++z_check) {
    if (isWithinBounds({x, y, z_check}) && isOccupied({x, y, z_check})) {
      return false; // Found an occupied cell
    }
  }
  return true; // No occupied cells found above
}

bool AstarOctoPlanner::isCylinderCollisionFree(const std::tuple<int, int, int>& coord, double radius) 
{
  int x, y, z;
  std::tie(x, y, z) = coord;

  double grid_radius = radius / voxel_size_;
  int grid_z_start = static_cast<int>(0.4 / voxel_size_);
  int grid_z_end = static_cast<int>(0.6 / voxel_size_);

  int num_points = static_cast<int>(2 * M_PI * grid_radius);

  for (int angle = 0; angle < num_points; angle += 2) {
    double theta = 2 * M_PI * angle / num_points;
    int i = static_cast<int>(std::round(grid_radius * std::cos(theta)));
    int j = static_cast<int>(std::round(grid_radius * std::sin(theta)));

    for (int k = grid_z_start; k <= grid_z_end; k += 2) {
      std::tuple<int, int, int> check_coord = {x + i, y + j, z + k};

      if (isWithinBounds(check_coord) && isOccupied(check_coord)) {
        return false; // Collision detected
      }
    }
  }
  return true; // No collision
}

std::vector<std::tuple<int, int, int>> AstarOctoPlanner::astar(const std::tuple<int, int, int>& start,
                                                                const std::tuple<int, int, int>& goal)
{
  std::priority_queue<GridNode, std::vector<GridNode>, std::greater<GridNode>> open_set;
  std::map<std::tuple<int, int, int>, double> g_score;
  std::map<std::tuple<int, int, int>, std::tuple<int, int, int>> came_from;

  // Lambda for the Euclidean heuristic.
  auto heuristic = [this](const std::tuple<int, int, int>& a, const std::tuple<int, int, int>& b) -> double {
    int ax, ay, az, bx, by, bz;
    std::tie(ax, ay, az) = a;
    std::tie(bx, by, bz) = b;
    return std::sqrt((ax - bx) * (ax - bx) + (ay - by) * (ay - by) + (az - bz) * (az - bz));
  };

  open_set.push({start, heuristic(start, goal), 0.0});
  g_score[start] = 0.0;

  // Define neighbor offsets for a 26-connected grid.
  std::vector<std::tuple<int, int, int>> neighbor_offsets = {
    {1, 1, 1},   {1, 1, -1},  {1, -1, 1},  {1, -1, -1},
    {-1, 1, 1},  {-1, 1, -1}, {-1, -1, 1}, {-1, -1, -1},
    {1, 1, 0},   {1, -1, 0},  {-1, 1, 0},  {-1, -1, 0},
    {1, 0, 1},   {1, 0, -1},  {-1, 0, 1},  {-1, 0, -1},
    {0, 1, 1},   {0, 1, -1},  {0, -1, 1},  {0, -1, -1},
    {1, 0, 0},   {-1, 0, 0},
    {0, 1, 0},   {0, -1, 0},
    {0, 0, 1},   {0, 0, -1}
  };

  while (!open_set.empty()) {
    auto current = open_set.top();
    open_set.pop();

    if (current.coord == goal) {
      std::vector<std::tuple<int, int, int>> path;
      auto node = current.coord;
      while (came_from.find(node) != came_from.end()) {
        path.push_back(node);
        node = came_from[node];
      }
      path.push_back(start);
      std::reverse(path.begin(), path.end());
      return path;
    }

    // Examine neighbors.
    for (const auto& offset : neighbor_offsets) {
      int dx, dy, dz;
      std::tie(dx, dy, dz) = offset;
      int cx, cy, cz;
      std::tie(cx, cy, cz) = current.coord;
      std::tuple<int, int, int> neighbor = {cx + dx, cy + dy, cz + dz};

      if (!isWithinBounds(neighbor))
        continue;

      if (!isOccupied(neighbor))
        continue;

      // Check if neighbor is within certain Z distance
      int current_z, neighbor_z;
      std::tie(std::ignore, std::ignore, current_z) = current.coord;
      std::tie(std::ignore, std::ignore, neighbor_z) = neighbor;
      int z_diff = std::abs(neighbor_z - current_z);
      int z_constraint = static_cast<int>(z_threshold_ / voxel_size_);
      if (z_diff > z_constraint)
        continue;

      if (!isCylinderCollisionFree(neighbor, robot_radius_))
        continue;

      if (!hasNoOccupiedCellsAbove(neighbor, min_vertical_clearance_, max_vertical_clearance_))
        continue;

      double tentative_g = current.g + heuristic(current.coord, neighbor);
      if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
        came_from[neighbor] = current.coord;
        g_score[neighbor] = tentative_g;
        double f = tentative_g + heuristic(neighbor, goal);
        open_set.push({neighbor, f, tentative_g});
      }
    }
  }

  return {};  // Return empty path if no solution is found.
}

void AstarOctoPlanner::pointcloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
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

  // Create a subscription to the point cloud topic.
  pointcloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/octomap_point_cloud_centers", 1,
      std::bind(&AstarOctoPlanner::pointcloud2Callback, this, std::placeholders::_1));

  reconfiguration_callback_handle_ = node_->add_on_set_parameters_callback(
      std::bind(&AstarOctoPlanner::reconfigureCallback, this, std::placeholders::_1));

  return true;
}

rcl_interfaces::msg::SetParametersResult AstarOctoPlanner::reconfigureCallback(std::vector<rclcpp::Parameter> parameters)
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

} // namespace astar_octo_planner
