// SPDX-License-Identifier: Apache-2.0
// Simplified octomap-backed A* planner implementing mbf_octo_core::OctoPlanner.

#include "octo_astar_planner/octo_astar_planner.hpp"

#include <octomap_msgs/conversions.h>

#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_set>

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(astar_octo_planner::AstarOctoPlanner, mbf_octo_core::OctoPlanner)

namespace astar_octo_planner
{

// ── initialize ───────────────────────────────────────────────────────────────

bool AstarOctoPlanner::initialize(const std::string & name,
                                   const rclcpp::Node::SharedPtr & node)
{
  name_ = name;
  node_ = node;

  // Helper: declare param if not yet declared, then read it.
  auto declare_string = [&](const std::string & key, const std::string & def) {
    if (!node_->has_parameter(key)) node_->declare_parameter(key, def);
    return node_->get_parameter(key).as_string();
  };
  auto declare_double = [&](const std::string & key, double def) {
    if (!node_->has_parameter(key)) node_->declare_parameter(key, def);
    return node_->get_parameter(key).as_double();
  };

  // Spot physical params (Boston Dynamics Spot):
  //   body 1.1 m × 0.5 m × 0.60 m standing height, ground clearance ~0.17 m,
  //   circumscribed footprint radius ≈ 0.60 m, max slope 30° (tuned to 50°).
  const std::string ns = name_ + ".";
  octomap_topic_      = declare_string(ns + "octomap_topic",      "/octomap_binary");
  resolution_         = declare_double(ns + "resolution",         0.1);   // m — grid cell size
  robot_height_       = declare_double(ns + "robot_height",       0.65);  // m — Spot body height + 5 cm margin
  robot_radius_       = declare_double(ns + "robot_radius",       0.55);  // m — conservative circumscribed radius
  max_ray_drop_       = declare_double(ns + "max_ray_drop",       3.0);   // m — max downward ray for ground search
  max_slope_deg_      = declare_double(ns + "max_slope_deg",      50.0);  // deg — max traversable terrain slope
  anomaly_filter_enabled_ = [&]() {
    if (!node_->has_parameter(ns + "anomaly_filter_enabled")) node_->declare_parameter(ns + "anomaly_filter_enabled", true);
    return node_->get_parameter(ns + "anomaly_filter_enabled").as_bool();
  }();
  anomaly_min_neighbors_ = [&]() {
    if (!node_->has_parameter(ns + "anomaly_min_neighbors")) node_->declare_parameter(ns + "anomaly_min_neighbors", 2);
    return static_cast<int>(node_->get_parameter(ns + "anomaly_min_neighbors").as_int());
  }();
  map_frame_          = declare_string(ns + "map_frame",          "map");

  octomap_sub_ = node_->create_subscription<octomap_msgs::msg::Octomap>(
    "/navigation/octomap_binary",
    rclcpp::QoS(1).transient_local(),
    std::bind(&AstarOctoPlanner::octomapCallback, this, std::placeholders::_1));

  path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    name_ + "/path", rclcpp::QoS(1).transient_local());

  traversable_cells_pub_ = node_->create_publisher<octomap_msgs::msg::Octomap>(
    name_ + "/traversable_cells", rclcpp::QoS(1).transient_local());

  RCLCPP_INFO(node_->get_logger(),
    "[%s] Initialized (Spot) — octomap_topic=%s height=%.2f m radius=%.2f m "
    "max_slope=%.1f° (z_offset=%.3f m) max_ray_drop=%.2f m res=%.3f m anomaly_filter=%s(min_nb=%d)",
    name_.c_str(), octomap_topic_.c_str(),
    robot_height_, robot_radius_,
    max_slope_deg_, robot_radius_ * std::tan(max_slope_deg_ * M_PI / 180.0),
    max_ray_drop_, resolution_,
    anomaly_filter_enabled_ ? "on" : "off", anomaly_min_neighbors_);
  return true;
}

// ── cancel ───────────────────────────────────────────────────────────────────

bool AstarOctoPlanner::cancel()
{
  cancel_planning_ = true;
  return true;
}

// ── octomapCallback ───────────────────────────────────────────────────────────

void AstarOctoPlanner::octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
  octomap::AbstractOcTree * tree_ptr =
    msg->binary ? octomap_msgs::binaryMsgToMap(*msg) : octomap_msgs::fullMsgToMap(*msg);

  auto * oct = dynamic_cast<octomap::OcTree *>(tree_ptr);
  if (!oct) {
    delete tree_ptr;
    RCLCPP_WARN(node_->get_logger(), "[%s] Received non-OcTree octomap — ignored.", name_.c_str());
    return;
  }

  std::lock_guard<std::mutex> lock(octree_mutex_);
  octree_.reset(oct);
  resolution_ = octree_->getResolution();
  RCLCPP_DEBUG(node_->get_logger(), "[%s] Octomap updated (res=%.3f m).", name_.c_str(), resolution_);
}

// ── Grid helpers ──────────────────────────────────────────────────────────────

double AstarOctoPlanner::heuristic(int x1, int y1, int x2, int y2) const
{
  const double dx = static_cast<double>(x1 - x2) * resolution_;
  const double dy = static_cast<double>(y1 - y2) * resolution_;
  return std::hypot(dx, dy);
}

AstarOctoPlanner::GridCell AstarOctoPlanner::worldToGrid(double wx, double wy) const
{
  return {
    static_cast<int>(std::round(wx / resolution_)),
    static_cast<int>(std::round(wy / resolution_))
  };
}

std::pair<double, double> AstarOctoPlanner::gridToWorld(int gx, int gy) const
{
  return {gx * resolution_, gy * resolution_};
}

// ── Anomaly filter ────────────────────────────────────────────────────────────

bool AstarOctoPlanner::isAnomalyVoxel(double wx, double wy, double wz) const
{
  if (!anomaly_filter_enabled_) return false;
  // Count occupied 6-connected face neighbors (±x, ±y, ±z at one voxel step).
  const double r = resolution_;
  const double offsets[6][3] = {
    { r, 0, 0}, {-r, 0, 0},
    { 0, r, 0}, { 0,-r, 0},
    { 0, 0, r}, { 0, 0,-r}
  };
  int occupied_neighbors = 0;
  for (const auto & off : offsets) {
    const auto * nb = octree_->search(wx + off[0], wy + off[1], wz + off[2]);
    if (nb && octree_->isNodeOccupied(*nb)) ++occupied_neighbors;
  }
  return occupied_neighbors < anomaly_min_neighbors_;
}

// ── Octree queries ────────────────────────────────────────────────────────────

std::vector<octomap::point3d> AstarOctoPlanner::cellFloorZ(int gx, int gy) const
{
  const auto [wx, wy] = gridToWorld(gx, gy);

  double omx, omy, map_min_z, omX, omY, map_max_z;
  octree_->getMetricMin(omx, omy, map_min_z);
  octree_->getMetricMax(omX, omY, map_max_z);

  const octomap::point3d col_min(
    static_cast<float>(wx - resolution_ * 0.4),
    static_cast<float>(wy - resolution_ * 0.4),
    static_cast<float>(map_min_z));
  const octomap::point3d col_max(
    static_cast<float>(wx + resolution_ * 0.4),
    static_cast<float>(wy + resolution_ * 0.4),
    static_cast<float>(map_max_z));

  // Collect actual voxel centers from the source octree for all occupied voxels
  // that pass radial clearance.  Using iterator coordinates (not grid-derived wx/wy)
  // ensures published traversable cells map 1-to-1 with source octomap voxels.
  std::vector<octomap::point3d> floor_voxels;
  for (auto it  = octree_->begin_leafs_bbx(col_min, col_max),
            end = octree_->end_leafs_bbx(); it != end; ++it) {
    if (!octree_->isNodeOccupied(*it)) continue;
    const octomap::point3d vox = it.getCoordinate();
    const double z_floor = static_cast<double>(vox.z()) + resolution_ * 0.5;
    if (checkRadialClearance(wx, wy, z_floor)) {
      floor_voxels.push_back(vox);
    }
  }
  return floor_voxels;
}

bool AstarOctoPlanner::isCellTraversable(int gx, int gy) const
{
  return !cellFloorZ(gx, gy).empty();
}

AstarOctoPlanner::GridCell AstarOctoPlanner::findNearestTraversableCell(
  int gx, int gy, int max_search_radius) const
{
  if (isCellTraversable(gx, gy)) {
    return {gx, gy};
  }
  for (int radius = 1; radius <= max_search_radius; ++radius) {
    for (int dx = -radius; dx <= radius; ++dx) {
      for (int dy = -radius; dy <= radius; ++dy) {
        if (std::abs(dx) != radius && std::abs(dy) != radius) continue;
        if (isCellTraversable(gx + dx, gy + dy)) {
          return {gx + dx, gy + dy};
        }
      }
    }
  }
  return {gx, gy};
}

bool AstarOctoPlanner::checkRadialClearance(double wx, double wy, double z_floor) const
{
  // Use octomap's BBX leaf iterator — prunes subtrees outside the box in one pass,
  // faster than 8-direction × N-slice individual octree_->search() calls.
  // z_offset: on a slope of max_slope_deg_, a floor voxel at the far edge of the
  // robot footprint (robot_radius_ away) sits up to robot_radius_*tan(slope) above
  // the centre floor voxel.  Use robot_radius_ (not resolution_) so those sloped
  // floor voxels are excluded from the clearance window and not counted as obstacles.
  const double z_offset = robot_radius_ * std::tan(max_slope_deg_ * M_PI / 180.0);
  const double z_start = z_floor + z_offset;
  const double z_end   = z_floor + robot_height_;

  const octomap::point3d bbx_min(
    static_cast<float>(wx - robot_radius_),
    static_cast<float>(wy - robot_radius_),
    static_cast<float>(z_start));
  const octomap::point3d bbx_max(
    static_cast<float>(wx + robot_radius_),
    static_cast<float>(wy + robot_radius_),
    static_cast<float>(z_end));

  for (auto it  = octree_->begin_leafs_bbx(bbx_min, bbx_max),
            end = octree_->end_leafs_bbx();
       it != end; ++it)
  {
    if (!octree_->isNodeOccupied(*it)) continue;
    const octomap::point3d c = it.getCoordinate();
    if (!isAnomalyVoxel(
          static_cast<double>(c.x()),
          static_cast<double>(c.y()),
          static_cast<double>(c.z())))
    {
      return false;
    }
  }
  return true;
}

// ── Path reconstruction ───────────────────────────────────────────────────────

std::vector<geometry_msgs::msg::PoseStamped> AstarOctoPlanner::reconstructPath(
  const std::unordered_map<GridCell, GridCell, GridCellHash> & came_from,
  GridCell current,
  const std::string & frame_id) const
{
  std::vector<GridCell> cells;
  for (auto it = came_from.find(current);
       it != came_from.end();
       it = came_from.find(current))
  {
    cells.push_back(current);
    current = it->second;
  }
  cells.push_back(current);  // start cell
  std::reverse(cells.begin(), cells.end());

  const rclcpp::Time stamp = node_->now();
  std::vector<geometry_msgs::msg::PoseStamped> path;
  path.reserve(cells.size());

  for (const auto & cell : cells) {
    const auto [wx, wy] = gridToWorld(cell.first, cell.second);
    const auto voxels = cellFloorZ(cell.first, cell.second);
    double z = 0.0;
    if (!voxels.empty()) {
      const auto min_it = std::min_element(voxels.begin(), voxels.end(),
        [](const octomap::point3d & a, const octomap::point3d & b) { return a.z() < b.z(); });
      z = static_cast<double>(min_it->z()) + resolution_ * 0.5 + robot_height_ * 0.5;
    }

    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = frame_id;
    p.header.stamp    = stamp;
    p.pose.position.x = wx;
    p.pose.position.y = wy;
    p.pose.position.z = z;
    p.pose.orientation.w = 1.0;
    path.push_back(p);
  }
  return path;
}

// ── makePlan ─────────────────────────────────────────────────────────────────

uint32_t AstarOctoPlanner::makePlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  double /*tolerance*/,
  std::vector<geometry_msgs::msg::PoseStamped> & plan,
  double & cost,
  std::string & message)
{
  cancel_planning_ = false;
  plan.clear();

  std::unique_lock<std::mutex> lock(octree_mutex_);
  if (!octree_) {
    message = "No octomap received yet.";
    RCLCPP_WARN(node_->get_logger(), "[%s] %s", name_.c_str(), message.c_str());
    return mbf_msgs::action::GetPath::Result::FAILURE;
  }

  const std::string frame_id =
    map_frame_.empty() ? start.header.frame_id : map_frame_;

  // search_z: ray origin for all downward ground-finding casts.
  // Must be ABOVE the highest terrain point in the map, not just above the start pose.
  // Using a fixed offset from start Z causes cells at higher elevation to miss the
  // floor (returns NaN → fallback Z → sudden jump).  Query octree metric max instead.
  double map_max_z = start.pose.position.z + max_ray_drop_;
  {
    double mx, my, mz;
    octree_->getMetricMax(mx, my, mz);
    map_max_z = std::max(map_max_z, mz);
  }
  const double search_z = map_max_z + 0.5;

  const GridCell start_cell =
    worldToGrid(start.pose.position.x, start.pose.position.y);
  const GridCell goal_cell  =
    worldToGrid(goal.pose.position.x,  goal.pose.position.y);

  // Snap start and goal to nearest traversable cells if needed
  const GridCell snapped_start = findNearestTraversableCell(start_cell.first, start_cell.second);
  const GridCell snapped_goal  = findNearestTraversableCell(goal_cell.first,  goal_cell.second);

  // Pre-populate traversable voxel cache: value is actual source voxel centers (empty = not traversable).
  std::unordered_map<GridCell, std::vector<octomap::point3d>, GridCellHash> full_trav_cache;
  {
    const int min_gx = std::min(snapped_start.first, snapped_goal.first) - 100;
    const int max_gx = std::max(snapped_start.first, snapped_goal.first) + 100;
    const int min_gy = std::min(snapped_start.second, snapped_goal.second) - 100;
    const int max_gy = std::max(snapped_start.second, snapped_goal.second) + 100;
    for (int gx = min_gx; gx <= max_gx; ++gx) {
      for (int gy = min_gy; gy <= max_gy; ++gy) {
        GridCell c{gx, gy};
        if (full_trav_cache.find(c) == full_trav_cache.end()) {
          full_trav_cache[c] = cellFloorZ(gx, gy);
        }
      }
    }
  }

  if (snapped_start == snapped_goal) {
    plan.push_back(goal);
    cost = 0.0;
    
    // Still publish traversable cells
    {
      auto trav_map = std::make_unique<octomap::OcTree>(octree_->getResolution());
      for (const auto & [cell, voxels] : full_trav_cache) {
        for (const auto & vox : voxels) {
          trav_map->updateNode(vox, true);
        }
      }
      trav_map->updateInnerOccupancy();
      octomap_msgs::msg::Octomap trav_msg;
      trav_msg.header.stamp    = node_->now();
      trav_msg.header.frame_id = frame_id;
      octomap_msgs::binaryMapToMsg(*trav_map, trav_msg);
      traversable_cells_pub_->publish(trav_msg);
    }
    
    return mbf_msgs::action::GetPath::Result::SUCCESS;
  }

  RCLCPP_INFO(node_->get_logger(),
    "[%s] Planning from grid (%d,%d) to (%d,%d)",
    name_.c_str(), snapped_start.first, snapped_start.second,
    snapped_goal.first, snapped_goal.second);

  // ── A* ───────────────────────────────────────────────────────────────────

  struct OpenItem {
    double f;
    GridCell cell;
    bool operator>(const OpenItem & o) const { return f > o.f; }
  };
  std::priority_queue<OpenItem, std::vector<OpenItem>, std::greater<OpenItem>> open_set;

  std::unordered_map<GridCell, GridCell, GridCellHash> came_from;
  std::unordered_map<GridCell, double,   GridCellHash> g_score;
  std::unordered_set<GridCell,           GridCellHash> closed_set;

  // Cache: actual source voxel centers per cell (empty = not traversable). Seeded from pre-computed map.
  std::unordered_map<GridCell, std::vector<octomap::point3d>, GridCellHash> trav_cache = full_trav_cache;
  auto isTraversable = [&](int gx, int gy) -> bool {
    GridCell c{gx, gy};
    auto it = trav_cache.find(c);
    if (it != trav_cache.end()) return !it->second.empty();
    auto voxels = cellFloorZ(gx, gy);
    trav_cache[c] = voxels;
    return !voxels.empty();
  };

  const double h0 = heuristic(snapped_start.first, snapped_start.second,
                               snapped_goal.first,  snapped_goal.second);
  open_set.push({h0, snapped_start});
  g_score[snapped_start] = 0.0;

  // 8-connected neighbour offsets and their step costs (scaled to world distance)
  static const int DX[8]    = { 1, -1,  0,  0,  1, -1,  1, -1};
  static const int DY[8]    = { 0,  0,  1, -1,  1, -1, -1,  1};
  static const double SC[8] = { 1,  1,  1,  1,
                                 1.41421356, 1.41421356,
                                 1.41421356, 1.41421356 };

  while (!open_set.empty()) {
    if (cancel_planning_) {
      message = "Planning cancelled.";
      return mbf_msgs::action::GetPath::Result::CANCELED;
    }

    const auto [f, current] = open_set.top();
    open_set.pop();

    if (closed_set.count(current)) continue;
    closed_set.insert(current);

    if (current == snapped_goal) {
      plan = reconstructPath(came_from, current, frame_id);
      cost = static_cast<double>(plan.size()) * resolution_;

      nav_msgs::msg::Path path_msg;
      path_msg.header.stamp    = node_->now();
      path_msg.header.frame_id = frame_id;
      path_msg.poses           = plan;
      path_pub_->publish(path_msg);

      RCLCPP_INFO(node_->get_logger(),
        "[%s] Path found: %zu waypoints, cost=%.2f m",
        name_.c_str(), plan.size(), cost);
      return mbf_msgs::action::GetPath::Result::SUCCESS;
    }

    const double cur_g = g_score.count(current) ? g_score.at(current)
                                                 : std::numeric_limits<double>::infinity();

    for (int i = 0; i < 8; ++i) {
      GridCell nb{current.first + DX[i], current.second + DY[i]};
      if (closed_set.count(nb)) continue;
      // Always allow the explicit goal cell even if its traversability query
      // fails (goal may be at an obstacle edge; tolerance handling is upstream).
      if (nb != snapped_goal && !isTraversable(nb.first, nb.second)) continue;

      const double tentative_g = cur_g + SC[i] * resolution_;
      const auto git = g_score.find(nb);
      if (git == g_score.end() || tentative_g < git->second) {
        came_from[nb] = current;
        g_score[nb]   = tentative_g;
        const double h = heuristic(nb.first, nb.second, snapped_goal.first, snapped_goal.second);
        open_set.push({tentative_g + h, nb});
      }
    }
  }

  // Publish octomap of traversable cells for visualization (always, success or not)
  {
    auto trav_map = std::make_unique<octomap::OcTree>(octree_->getResolution());
    for (const auto & [cell, voxels] : trav_cache) {
      for (const auto & vox : voxels) {
        trav_map->updateNode(vox, true);
      }
    }
    trav_map->updateInnerOccupancy();
    octomap_msgs::msg::Octomap trav_msg;
    trav_msg.header.stamp    = node_->now();
    trav_msg.header.frame_id = frame_id;
    octomap_msgs::binaryMapToMsg(*trav_map, trav_msg);
    traversable_cells_pub_->publish(trav_msg);
  }

  message = "No path found between start and goal.";
  RCLCPP_WARN(node_->get_logger(),
    "[%s] No path found (start=(%d,%d) goal=(%d,%d), expanded=%zu).",
    name_.c_str(), snapped_start.first, snapped_start.second,
    snapped_goal.first, snapped_goal.second, closed_set.size());
  return mbf_msgs::action::GetPath::Result::NO_PATH_FOUND;
}

}  // namespace astar_octo_planner
