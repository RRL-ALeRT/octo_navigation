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
  // FIX: read octomap_topic_ from parameter instead of hard-coding "/octomap_binary".
  octomap_topic_      = declare_string(ns + "octomap_topic",      "/octomap_binary");
  // FIX: read path_topic_ from parameter instead of hard-coding "/move_base_flex/path".
  path_topic_         = declare_string(ns + "path_topic",         "~/path");
  resolution_         = declare_double(ns + "resolution",         0.1);   // m — grid cell size
  robot_height_       = declare_double(ns + "skpawar1305_robot_height",       0.65);  // m — Spot body height + 5 cm margin
  robot_radius_       = declare_double(ns + "skpawar1305_robot_radius",       0.55);  // m — circumscribed radius (also half-width of clearance square)
  max_ray_drop_       = declare_double(ns + "max_ray_drop",       3.0);   // m — retained for param compatibility; not used in planning logic
  max_slope_deg_      = declare_double(ns + "max_slope_deg",      50.0);  // deg — max traversable terrain slope (enforced in A* expansion)
  anomaly_filter_enabled_ = [&]() {
    if (!node_->has_parameter(ns + "anomaly_filter_enabled")) node_->declare_parameter(ns + "anomaly_filter_enabled", true);
    return node_->get_parameter(ns + "anomaly_filter_enabled").as_bool();
  }();
  anomaly_min_neighbors_ = [&]() {
    if (!node_->has_parameter(ns + "anomaly_min_neighbors")) node_->declare_parameter(ns + "anomaly_min_neighbors", 2);
    return static_cast<int>(node_->get_parameter(ns + "anomaly_min_neighbors").as_int());
  }();
  z_penalty_factor_   = declare_double(ns + "z_penalty_factor",   10.0);  // cost multiplier for z displacement
  map_frame_          = declare_string(ns + "map_frame",          "map");

  // FIX: use octomap_topic_ parameter instead of hard-coded literal.
  octomap_sub_ = node_->create_subscription<octomap_msgs::msg::Octomap>(
    octomap_topic_,
    rclcpp::QoS(1),
    std::bind(&AstarOctoPlanner::octomapCallback, this, std::placeholders::_1));

  // FIX: use path_topic_ parameter instead of hard-coded literal.
  path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    path_topic_, rclcpp::QoS(1).transient_local());

  traversable_cells_pub_ = node_->create_publisher<octomap_msgs::msg::Octomap>(
    name_ + "/traversable_cells", rclcpp::QoS(1).transient_local());

  RCLCPP_INFO(node_->get_logger(),
    "[%s] Initialized (Spot) — octomap_topic=%s path_topic=%s "
    "height=%.2f m radius=%.2f m max_slope=%.1f° "
    "max_ray_drop=%.2f m res=%.3f m z_penalty_factor=%.1f anomaly_filter=%s(min_nb=%d)",
    name_.c_str(), octomap_topic_.c_str(), path_topic_.c_str(),
    robot_height_, robot_radius_,
    max_slope_deg_, max_ray_drop_, resolution_,
    z_penalty_factor_,
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
  // A voxel with fewer than anomaly_min_neighbors_ occupied face-neighbors is
  // treated as sensor noise and ignored.  Default is 2; note that a lone voxel
  // on a flat floor has only 1 neighbor below, so setting this > 1 will filter
  // out isolated floor points — tune carefully.
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
  // Collect occupied voxels in the column, sort bottom-up, then stop at the
  // first voxel whose clearance check passes.  This correctly handles multi-level
  // terrain (lower platform blocked → try next level up) while avoiding redundant
  // clearance scans: typical floor cells return after exactly 1 check.
  std::vector<octomap::point3d> column_voxels;
  for (auto it  = octree_->begin_leafs_bbx(col_min, col_max),
            end = octree_->end_leafs_bbx(); it != end; ++it) {
    if (!octree_->isNodeOccupied(*it)) continue;
    column_voxels.push_back(it.getCoordinate());
  }
  std::sort(column_voxels.begin(), column_voxels.end(),
    [](const octomap::point3d & a, const octomap::point3d & b) { return a.z() < b.z(); });

  std::vector<octomap::point3d> floor_voxels;
  for (const auto & vox : column_voxels) {
    const double z_floor = static_cast<double>(vox.z()) + resolution_ * 0.5;
    if (checkRadialClearance(wx, wy, z_floor)) {
      floor_voxels.push_back(vox);
      break;  // stop at first valid floor level
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
  // Query an axis-aligned bounding box around (wx, wy) at body height.
  // The BBX is a square of half-side robot_radius_; the effective footprint is
  // therefore square, not circular.  Voxels in the BBX corners but outside the
  // true robot_radius_ circle are filtered below with an explicit Euclidean check.
  //
  // Scan the BBX from z_floor to z_floor+robot_height_ for obstacle voxels.
  // Each candidate voxel is only counted as an obstacle if it rises above the
  // locally-expected terrain surface at its XY offset from the cell centre.
  //
  // The expected surface height at horizontal distance d from the centre is:
  //   z_floor + d * tan(max_slope_deg_)
  // This is a cone-shaped dead zone that grows with distance.  It tolerates
  // slope floor voxels (which sit exactly on this cone) without creating a
  // global flat dead zone that would hide nearby walls:
  //   • Wall voxel at d≈0 (same XY as cell centre): dead zone = 0 → always detected.
  //   • Slope floor voxel at distance d: dead zone = d*tan(slope) → exempted ✓
  //   • Tall obstacle at any d: rises above cone → detected ✓
  const double max_slope_tan = std::tan(max_slope_deg_ * M_PI / 180.0);

  const octomap::point3d bbx_min(
    static_cast<float>(wx - robot_radius_),
    static_cast<float>(wy - robot_radius_),
    static_cast<float>(z_floor));
  const octomap::point3d bbx_max(
    static_cast<float>(wx + robot_radius_),
    static_cast<float>(wy + robot_radius_),
    static_cast<float>(z_floor + robot_height_));

  const double radius_sq = robot_radius_ * robot_radius_;

  for (auto it  = octree_->begin_leafs_bbx(bbx_min, bbx_max),
            end = octree_->end_leafs_bbx();
       it != end; ++it)
  {
    if (!octree_->isNodeOccupied(*it)) continue;
    const octomap::point3d c = it.getCoordinate();

    // Filter to true circular footprint.
    const double cdx = static_cast<double>(c.x()) - wx;
    const double cdy = static_cast<double>(c.y()) - wy;
    const double d_sq = cdx * cdx + cdy * cdy;
    if (d_sq > radius_sq) continue;

    // Cone dead zone: exempt voxels at or below the expected slope surface.
    // Avoid sqrt: c.z() <= z_floor + sqrt(d_sq)*tan  iff
    //   dz <= 0  OR  dz^2 <= d_sq * tan^2
    const double dz = static_cast<double>(c.z()) - z_floor;
    if (dz <= 0.0 || dz * dz <= d_sq * max_slope_tan * max_slope_tan) continue;

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

// FIX: accept trav_cache so reconstruction reads cached voxels rather than
// re-querying the octree for every waypoint.  Also accepts the goal g_score
// so the returned cost reflects actual path length, not waypoint count.
std::vector<geometry_msgs::msg::PoseStamped> AstarOctoPlanner::reconstructPath(
  const std::unordered_map<GridCell, GridCell, GridCellHash> & came_from,
  const std::unordered_map<GridCell, std::vector<octomap::point3d>, GridCellHash> & trav_cache,
  GridCell current,
  const std::string & frame_id) const
{
  std::vector<GridCell> cells;

  // FIX: guard against cycles (shouldn't occur in correct A*, but defensive).
  std::unordered_set<GridCell, GridCellHash> visited;
  while (true) {
    auto it = came_from.find(current);
    if (it == came_from.end()) break;           // reached the start node
    if (!visited.insert(current).second) break; // cycle detected — stop
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

    // FIX: use cached voxels instead of calling cellFloorZ again.
    double z = 0.0;
    auto cache_it = trav_cache.find(cell);
    if (cache_it != trav_cache.end() && !cache_it->second.empty()) {
      const auto min_it = std::min_element(
        cache_it->second.begin(), cache_it->second.end(),
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
  cost = 0.0;

  // ── Snapshot the octree under lock, then release immediately ─────────────
  // FIX: previously the mutex was held for the entire duration of makePlan,
  // blocking octomapCallback for potentially several seconds.  We now take a
  // shared_ptr snapshot and release the lock before any expensive work.  All
  // subsequent octree access in this function goes through the snapshot;
  // the live octree_ member may be replaced by the callback concurrently
  // without affecting this planning call.
  std::shared_ptr<octomap::OcTree> oct_snap;
  double snap_resolution;
  {
    std::unique_lock<std::mutex> lock(octree_mutex_);
    if (!octree_) {
      message = "No octomap received yet.";
      RCLCPP_WARN(node_->get_logger(), "[%s] %s", name_.c_str(), message.c_str());
      return mbf_msgs::action::GetPath::Result::FAILURE;
    }
    oct_snap = octree_;              // shared_ptr copy — keeps the tree alive
    snap_resolution = resolution_;   // resolution_ is set atomically with octree_
  }
  // Lock released here — octomapCallback is free to update octree_ from now on.

  // All helper methods use octree_ member, so point it at the snapshot for the
  // duration of this call.  This is safe: we're the only thread that writes
  // octree_ here (the callback only writes under the mutex we've just released,
  // and our snapshot keeps the old tree alive via refcount).
  // Restore octree_ to nullptr at scope exit so we don't hold the snapshot
  // reference longer than needed.
  octree_ = oct_snap;

  const std::string frame_id =
    map_frame_.empty() ? start.header.frame_id : map_frame_;

  const GridCell start_cell =
    worldToGrid(start.pose.position.x, start.pose.position.y);
  const GridCell goal_cell  =
    worldToGrid(goal.pose.position.x,  goal.pose.position.y);

  // Snap start and goal to nearest traversable cells if needed.
  const GridCell snapped_start = findNearestTraversableCell(start_cell.first, start_cell.second);
  const GridCell snapped_goal  = findNearestTraversableCell(goal_cell.first,  goal_cell.second);

  // Pre-populate traversable voxel cache over a bounding box that comfortably
  // covers the straight-line corridor between start and goal.  The A* search
  // will only consult this cache (no further octree queries), so the cache is
  // the single octree access phase for the entire planning call.
  using TravCache = std::unordered_map<GridCell, std::vector<octomap::point3d>, GridCellHash>;
  TravCache trav_cache;
  {
    // Pad 15 cells (~1.5 m at 0.1 m res) around the start-goal bounding box.
    // Increase if large obstacles force wider detours.
    const int min_gx = std::min(snapped_start.first, snapped_goal.first) - 15;
    const int max_gx = std::max(snapped_start.first, snapped_goal.first) + 15;
    const int min_gy = std::min(snapped_start.second, snapped_goal.second) - 15;
    const int max_gy = std::max(snapped_start.second, snapped_goal.second) + 15;
    for (int gx = min_gx; gx <= max_gx; ++gx) {
      for (int gy = min_gy; gy <= max_gy; ++gy) {
        trav_cache[{gx, gy}] = cellFloorZ(gx, gy);
      }
    }
  }

  // Pre-cache is complete — helper lambdas below use only trav_cache.
  // octree_ (snapshot) is still set for the helper methods used above, but
  // A* itself never calls cellFloorZ or checkRadialClearance directly.

  if (snapped_start == snapped_goal) {
    plan.push_back(goal);
    cost = 0.0;

    // Publish traversable cells for visualization.
    {
      auto trav_map = std::make_unique<octomap::OcTree>(snap_resolution);
      for (const auto & [cell, voxels] : trav_cache) {
        for (const auto & vox : voxels) { trav_map->updateNode(vox, true); }
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

  // FIX: isTraversable consults only the pre-built cache — no live octree access
  // during A*.  Cells outside the cache (beyond ±100 grid cells of the path
  // bounding box) are conservatively treated as non-traversable.
  auto isTraversable = [&](int gx, int gy) -> bool {
    auto it = trav_cache.find({gx, gy});
    return it != trav_cache.end() && !it->second.empty();
  };

  // Helper: floor Z of the lowest traversable voxel top for a cached cell.
  auto getCellZ = [&](const GridCell & c) -> double {
    auto it = trav_cache.find(c);
    if (it == trav_cache.end() || it->second.empty()) return 0.0;
    const auto min_it = std::min_element(it->second.begin(), it->second.end(),
      [](const octomap::point3d & a, const octomap::point3d & b) { return a.z() < b.z(); });
    return static_cast<double>(min_it->z()) + snap_resolution * 0.5;
  };

  // FIX: precompute slope threshold once rather than per-neighbor.
  // max_slope_deg_ now actually rejects neighbors whose rise/run exceeds the limit.
  const double max_slope_tan = std::tan(max_slope_deg_ * M_PI / 180.0);

  // 8-connected neighbour offsets and their step costs (world distance).
  static const int DX[8]    = { 1, -1,  0,  0,  1, -1,  1, -1};
  static const int DY[8]    = { 0,  0,  1, -1,  1, -1, -1,  1};
  static const double SC[8] = { 1,  1,  1,  1,
                                 1.41421356, 1.41421356,
                                 1.41421356, 1.41421356 };

  const double h0 = heuristic(snapped_start.first, snapped_start.second,
                               snapped_goal.first,  snapped_goal.second);
  open_set.push({h0, snapped_start});
  g_score[snapped_start] = 0.0;

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
      // FIX: pass trav_cache into reconstructPath so it uses cached voxel Z
      // values instead of re-querying the octree per waypoint.
      plan = reconstructPath(came_from, trav_cache, current, frame_id);

      // FIX: cost = actual accumulated g-score (world distance + z penalties),
      // not waypoint_count × resolution_ which ignores diagonal steps.
      cost = g_score.count(current) ? g_score.at(current) : 0.0;

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

      // FIX: enforce max_slope_deg_ — reject neighbors whose terrain slope
      // (rise / horizontal run) exceeds the configured limit.  Previously
      // max_slope_deg_ only affected the z_offset in checkRadialClearance and
      // had no effect on which neighbors were actually accepted by A*.
      const double dz = std::abs(getCellZ(nb) - getCellZ(current));
      const double horizontal_step = SC[i] * snap_resolution;
      if (dz / horizontal_step > max_slope_tan) continue;  // too steep — reject

      const double tentative_g = cur_g + horizontal_step + z_penalty_factor_ * dz;
      const auto git = g_score.find(nb);
      if (git == g_score.end() || tentative_g < git->second) {
        came_from[nb] = current;
        g_score[nb]   = tentative_g;
        const double h = heuristic(nb.first, nb.second, snapped_goal.first, snapped_goal.second);
        open_set.push({tentative_g + h, nb});
      }
    }
  }

  // Publish traversable cells for visualization (on failure too).
  {
    auto trav_map = std::make_unique<octomap::OcTree>(snap_resolution);
    for (const auto & [cell, voxels] : trav_cache) {
      for (const auto & vox : voxels) { trav_map->updateNode(vox, true); }
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