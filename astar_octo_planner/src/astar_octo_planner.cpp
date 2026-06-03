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

#include <astar_octo_planner/astar_octo_planner.h>

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>

#include <mbf_msgs/action/get_path.hpp>

#include <queue>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <sstream>

PLUGINLIB_EXPORT_CLASS(astar_octo_planner::AstarOctoPlanner, mbf_octo_core::OctoPlanner);

namespace astar_octo_planner
{

AstarOctoPlanner::AstarOctoPlanner()
{
}

AstarOctoPlanner::~AstarOctoPlanner()
{
}

bool AstarOctoPlanner::cancel()
{
  cancel_planning_ = true;
  return true;
}

bool AstarOctoPlanner::initialize(const std::string& plugin_name,
                                   const rclcpp::Node::SharedPtr& node,
                                   const mbf_octo_core::OctoMapper::Ptr& mapper)
{
  name_   = plugin_name;
  node_   = node;
  mapper_ = mapper;

  // ---- Publish/planning config ----
  config_.publish_vector_field = node_->declare_parameter(name_ + ".publish_vector_field", config_.publish_vector_field);
  config_.publish_face_vectors  = node_->declare_parameter(name_ + ".publish_face_vectors",  config_.publish_face_vectors);
  config_.goal_dist_offset      = node_->declare_parameter(name_ + ".goal_dist_offset",      config_.goal_dist_offset);
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Vertex cost limit.";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value   = 10.0;
    descriptor.floating_point_range.push_back(range);
    config_.cost_limit = node_->declare_parameter(name_ + ".cost_limit", config_.cost_limit, descriptor);
  }

  // ---- Path publishers ----
  path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    "~/path", rclcpp::QoS(1).transient_local());
  body_height_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    "~/body_height/path", rclcpp::QoS(1).transient_local());

  // ---- Robot collision parameters ----
  robot_radius_            = node_->declare_parameter(name_ + ".robot_radius",            robot_radius_);
  robot_width_             = node_->declare_parameter(name_ + ".robot_width",             robot_width_);
  robot_length_            = node_->declare_parameter(name_ + ".robot_length",            robot_length_);
  robot_height_            = node_->declare_parameter(name_ + ".robot_height",            robot_height_);
  footprint_margin_        = node_->declare_parameter(name_ + ".footprint_margin",        footprint_margin_);
  footprint_samples_x_     = node_->declare_parameter(name_ + ".footprint_samples_x",    footprint_samples_x_);
  footprint_samples_y_     = node_->declare_parameter(name_ + ".footprint_samples_y",    footprint_samples_y_);
  min_vertical_clearance_  = node_->declare_parameter(name_ + ".min_vertical_clearance", min_vertical_clearance_);
  max_vertical_clearance_  = node_->declare_parameter(name_ + ".max_vertical_clearance", max_vertical_clearance_);

  // ---- Live A* collision check parameters ----
  radial_clearance_num_dirs_   = node_->declare_parameter(name_ + ".radial_clearance_num_dirs",   radial_clearance_num_dirs_);
  radial_clearance_num_z_      = node_->declare_parameter(name_ + ".radial_clearance_num_z",      radial_clearance_num_z_);
  enable_radial_clearance_     = node_->declare_parameter(name_ + ".enable_radial_clearance",     enable_radial_clearance_);
  enable_edge_collision_check_ = node_->declare_parameter(name_ + ".enable_edge_collision_check", enable_edge_collision_check_);
  max_z_above_query_           = node_->declare_parameter(name_ + ".max_z_above_query",           max_z_above_query_);

  RCLCPP_INFO(node_->get_logger(),
    "AstarOctoPlanner: robot_height=%.2f, robot_radius=%.2f, min_vc=%.2f, max_vc=%.2f",
    robot_height_, robot_radius_, min_vertical_clearance_, max_vertical_clearance_);

  reconfiguration_callback_handle_ = node_->add_on_set_parameters_callback(
    std::bind(&AstarOctoPlanner::reconfigureCallback, this, std::placeholders::_1));

  return true;
}

// ---------------------------------------------------------------------------
// makePlan
// ---------------------------------------------------------------------------
uint32_t AstarOctoPlanner::makePlan(const geometry_msgs::msg::PoseStamped& start,
                                     const geometry_msgs::msg::PoseStamped& goal,
                                     double /*tolerance*/,
                                     std::vector<geometry_msgs::msg::PoseStamped>& plan,
                                     double& cost,
                                     std::string& message)
{
  RCLCPP_INFO(node_->get_logger(), "AstarOctoPlanner: makePlan called. start=(%.3f, %.3f, %.3f)",
              start.pose.position.x, start.pose.position.y, start.pose.position.z);

  if (!mapper_) {
    RCLCPP_WARN(node_->get_logger(), "Mapper not set; cannot plan.");
    return mbf_msgs::action::GetPath::Result::FAILURE;
  }

  const octomap::OcTree* octree = mapper_->getOctree();
  if (!octree) {
    RCLCPP_WARN(node_->get_logger(), "No octomap available yet; cannot plan.");
    return mbf_msgs::action::GetPath::Result::FAILURE;
  }

  // Convenience locals derived from mapper
  const double active_voxel_size = mapper_->getVoxelSize();
  const std::string map_frame    = mapper_->getMapFrame();
  std::array<double, 3> min_bound, max_bound;
  mapper_->getBounds(min_bound, max_bound);

  // Snap start/goal to active voxel grid to reduce floating-point noise
  geometry_msgs::msg::Point start_world = start.pose.position;
  geometry_msgs::msg::Point goal_world  = goal.pose.position;
  auto snap = [&](geometry_msgs::msg::Point& p) {
    p.x = std::round(p.x / active_voxel_size) * active_voxel_size;
    p.y = std::round(p.y / active_voxel_size) * active_voxel_size;
    p.z = std::round(p.z / active_voxel_size) * active_voxel_size;
  };
  snap(start_world);
  snap(goal_world);
  RCLCPP_INFO(node_->get_logger(), "Snapped start: (%.3f, %.3f, %.3f)",
              start_world.x, start_world.y, start_world.z);

  plan.clear();
  const std::string path_frame = map_frame.empty() ? start.header.frame_id : map_frame;
  const rclcpp::Time now = node_->now();

  // Clear stale debug markers from the previous failed plan
  {
    visualization_msgs::msg::MarkerArray clear_ma;
    visualization_msgs::msg::Marker del;
    del.header.frame_id = map_frame.empty() ? "map" : map_frame;
    del.header.stamp    = now;
    del.ns              = "failed_expanded";
    del.id              = 0;
    del.action          = visualization_msgs::msg::Marker::DELETE;
    clear_ma.markers.push_back(del);
    mapper_->publishAdditionalMarkers(clear_ma);
  }

  // Get a planning-ready graph snapshot (revalidated + penalties pre-computed by mapper)
  auto planning_graph = mapper_->getReadyGraph();
  if (!planning_graph || planning_graph->empty()) {
    RCLCPP_WARN(node_->get_logger(), "Graph not ready yet; cannot plan.");
    message = "Graph not ready";
    return mbf_msgs::action::GetPath::Result::NO_PATH_FOUND;
  }
  RCLCPP_INFO(node_->get_logger(), "Planning on graph with %zu nodes.", planning_graph->size());

  // ---- Direction-aware start search: offset toward the goal side, probe floor ----
  // Shifts the start search XY toward the goal (front or back) by ~half the body
  // length so the first path node is already on the correct side of the robot.
  // Floor Z is estimated by raycasting from center + the directional offset; we keep
  // the highest hit Z (= nearest surface below) to handle variable body height and
  // inclined terrain. Works even if no voxels exist directly under the robot yet.
  const auto& sq = start.pose.orientation;
  double siny_cosp = 2.0 * (sq.w * sq.z + sq.x * sq.y);
  double cosy_cosp = 1.0 - 2.0 * (sq.y * sq.y + sq.z * sq.z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);

  const double cos_yaw      = std::cos(yaw);
  const double sin_yaw      = std::sin(yaw);
  const double probe_offset = 0.55;  // ~half body length, where feet are typically scanned

  // Is the goal in front of or behind the robot?
  const double goal_dx     = goal_world.x - start_world.x;
  const double goal_dy     = goal_world.y - start_world.y;
  const double forward_dot = goal_dx * cos_yaw + goal_dy * sin_yaw;
  const double sign        = (forward_dot >= 0.0) ? +1.0 : -1.0;  // +1 front, -1 back

  // Start search XY offset toward the goal side
  double start_search_x = start_world.x + sign * probe_offset * cos_yaw;
  double start_search_y = start_world.y + sign * probe_offset * sin_yaw;
  double start_search_z = start_world.z;

  // Probe center + directional offset for floor Z; keep the highest Z (nearest floor)
  double best_floor_z = -std::numeric_limits<double>::infinity();
  const std::array<std::pair<double,double>, 2> floor_probes = {{
    { 0.0,                              0.0                             },
    { sign * probe_offset * cos_yaw,    sign * probe_offset * sin_yaw  },
  }};
  const octomap::point3d ray_direction(0.0, 0.0, -1.0);
  for (const auto& [dx, dy] : floor_probes) {
    octomap::point3d ray_origin(start_world.x + dx, start_world.y + dy, start_world.z);
    octomap::point3d ray_hit;
    bool hit = octree->castRay(ray_origin, ray_direction, ray_hit,
                                /*ignoreUnknownCells=*/true, /*maxRange=*/5.0);
    if (hit) {
      const octomap::point3d hit_center = octree->keyToCoord(octree->coordToKey(ray_hit));
      if (hit_center.z() > best_floor_z) {
        best_floor_z   = hit_center.z();
        start_search_z = hit_center.z();
      }
    }
  }

  RCLCPP_INFO(node_->get_logger(),
    "Start search: goal is %s (dot=%.2f), offset %s by %.2fm -> search XY=(%.3f,%.3f) Z=%.3f",
    sign > 0 ? "in front" : "behind", forward_dot,
    sign > 0 ? "forward" : "backward", probe_offset,
    start_search_x, start_search_y, start_search_z);
  if (best_floor_z <= -std::numeric_limits<double>::infinity()) {
    RCLCPP_WARN(node_->get_logger(), "All floor raycasts missed - using robot pose Z as fallback.");
  }

  // ---- Snap start and goal to nearest walkable graph nodes ----
  std::string start_id = findClosestGraphNode(
    octomap::point3d(start_search_x, start_search_y, start_search_z), planning_graph);
  std::string goal_id  = findClosestGraphNode(
    octomap::point3d(goal_world.x,   goal_world.y,   goal_world.z),  planning_graph);

  // Re-snap start if snapped node has no walkable neighbors
  if (!start_id.empty()) {
    auto adj_it = planning_graph->adj.find(start_id);
    bool has_walkable_nb = false;
    if (adj_it != planning_graph->adj.end()) {
      for (const auto& nb : adj_it->second) {
        auto nit = planning_graph->nodes.find(nb);
        if (nit != planning_graph->nodes.end() && nit->second.is_walkable) {
          has_walkable_nb = true;
          break;
        }
      }
    }
    if (!has_walkable_nb) {
      RCLCPP_WARN(node_->get_logger(),
        "Start node %s has no walkable neighbors — searching for a connected start...",
        start_id.c_str());
      octomap::point3d sp(start_world.x, start_world.y, start_world.z);
      double best = std::numeric_limits<double>::infinity();
      std::string best_id;
      for (const auto& kv : planning_graph->nodes) {
        if (!kv.second.is_walkable) continue;
        auto a_it = planning_graph->adj.find(kv.first);
        if (a_it == planning_graph->adj.end()) continue;
        bool connected = false;
        for (const auto& nb : a_it->second) {
          auto nit = planning_graph->nodes.find(nb);
          if (nit != planning_graph->nodes.end() && nit->second.is_walkable) { connected = true; break; }
        }
        if (!connected) continue;
        double dx = kv.second.center.x() - sp.x();
        double dy = kv.second.center.y() - sp.y();
        double dz = kv.second.center.z() - sp.z();
        double d  = dx*dx + dy*dy + dz*dz;
        if (d < best) { best = d; best_id = kv.first; }
      }
      if (!best_id.empty()) {
        const auto& cn = planning_graph->nodes.at(best_id);
        RCLCPP_WARN(node_->get_logger(), "Re-snapped start to %s (%.3f, %.3f, %.3f)",
          best_id.c_str(), cn.center.x(), cn.center.y(), cn.center.z());
        start_id = best_id;
      }
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Graph search: start='%s' goal='%s'",
              start_id.empty() ? "<none>" : start_id.c_str(),
              goal_id.empty()  ? "<none>" : goal_id.c_str());

  // ---- Validate node centers are within map bounds ----
  auto valid_center = [&](const octomap::point3d& c) -> bool {
    if (!std::isfinite(c.x()) || !std::isfinite(c.y()) || !std::isfinite(c.z())) return false;
    if (c.x() < min_bound[0] - 1e-6 || c.x() > max_bound[0] + 1e-6) return false;
    if (c.y() < min_bound[1] - 1e-6 || c.y() > max_bound[1] + 1e-6) return false;
    if (c.z() < min_bound[2] - 1e-6 || c.z() > max_bound[2] + 1e-6) return false;
    return true;
  };

  bool centers_ok = true;
  if (!start_id.empty()) {
    const auto& s = planning_graph->nodes.at(start_id);
    RCLCPP_INFO(node_->get_logger(), "Start node: (%.3f, %.3f, %.3f) walkable=%d clearance=%d",
                s.center.x(), s.center.y(), s.center.z(), s.is_walkable,
                hasVerticalClearance(s.center, s.size));
    centers_ok &= valid_center(s.center);
  }
  if (!goal_id.empty()) {
    const auto& g = planning_graph->nodes.at(goal_id);
    RCLCPP_INFO(node_->get_logger(), "Goal node:  (%.3f, %.3f, %.3f) walkable=%d",
                g.center.x(), g.center.y(), g.center.z(), g.is_walkable);
    centers_ok &= valid_center(g.center);

    // Re-snap goal if it is non-walkable or fails live vertical clearance.
    // Non-walkable goals arise when revalidation strips the snapped node (e.g. transient
    // occupancy above it in the live map). Search the nearest walkable node that passes
    // clearance, expanding the XY search radius until one is found.
    bool goal_needs_resnap = !g.is_walkable || !hasVerticalClearance(g.center, g.size);
    if (goal_needs_resnap) {
      RCLCPP_WARN(node_->get_logger(),
        "Goal node is %s (walkable=%d). Searching for nearest walkable alternative...",
        !g.is_walkable ? "non-walkable" : "clearance-blocked", g.is_walkable);
      double best = std::numeric_limits<double>::infinity();
      std::string alt_id;
      // Expand search radius until we find something (up to 3 m)
      for (double xy_tol = active_voxel_size * 3.0; xy_tol <= 3.0 && alt_id.empty(); xy_tol *= 2.0) {
        for (const auto& kv : planning_graph->nodes) {
          if (!kv.second.is_walkable) continue;
          if (!hasVerticalClearance(kv.second.center, kv.second.size)) continue;
          double dx = kv.second.center.x() - g.center.x();
          double dy = kv.second.center.y() - g.center.y();
          double dz = kv.second.center.z() - g.center.z();
          if (std::abs(dx) > xy_tol || std::abs(dy) > xy_tol) continue;
          double score = std::abs(dz) + 0.1 * std::sqrt(dx*dx + dy*dy);
          if (score < best) { best = score; alt_id = kv.first; }
        }
      }
      if (!alt_id.empty()) {
        const auto& ag = planning_graph->nodes.at(alt_id);
        RCLCPP_INFO(node_->get_logger(), "Re-snapped goal to %s (%.3f, %.3f, %.3f)",
          alt_id.c_str(), ag.center.x(), ag.center.y(), ag.center.z());
        goal_id = alt_id;
      }
    }
  }

  if (!centers_ok || start_id.empty() || goal_id.empty()) {
    RCLCPP_WARN(node_->get_logger(),
      "Invalid or missing start/goal nodes. Aborting plan.");
    message = "Graph path not available or invalid centers";
    return mbf_msgs::action::GetPath::Result::NO_PATH_FOUND;
  }

  // ---- A* search ----
  const auto& local_nodes        = planning_graph->nodes;
  const auto& local_adj          = planning_graph->adj;
  const auto& local_node_penalty = planning_graph->node_penalty;

  std::unordered_set<std::string> expanded_set;
  std::unordered_map<std::string, double> gscore;
  std::unordered_map<std::string, std::string> came_from;

  struct PQItem { std::string id; double f; double g; };
  struct PQCmp  { bool operator()(const PQItem& a, const PQItem& b) const { return a.f > b.f; } };
  std::priority_queue<PQItem, std::vector<PQItem>, PQCmp> openq;

  auto heuristic = [&](const std::string& a) -> double {
    auto ait = local_nodes.find(a);
    auto git = local_nodes.find(goal_id);
    if (ait == local_nodes.end() || git == local_nodes.end()) return 0.0;
    double dx = ait->second.center.x() - git->second.center.x();
    double dy = ait->second.center.y() - git->second.center.y();
    double dz = ait->second.center.z() - git->second.center.z();
    return std::sqrt(dx*dx + dy*dy + dz*dz);
  };

  auto getPrecomputedPenalty = [&](const std::string& nid) -> double {
    auto it = local_node_penalty.find(nid);
    return it != local_node_penalty.end() ? it->second : 0.0;
  };

  openq.push({start_id, heuristic(start_id), 0.0});
  gscore[start_id] = 0.0;

  std::vector<std::string> path_ids;
  auto t_astar_start = std::chrono::steady_clock::now();
  size_t rejected_not_walkable       = 0;
  size_t rejected_vertical_clearance = 0;
  size_t rejected_radial_clearance   = 0;
  size_t rejected_edge_collision     = 0;
  size_t accepted_neighbors          = 0;

  while (!openq.empty()) {
    PQItem cur = openq.top(); openq.pop();
    if (cur.id == goal_id) {
      std::string u = goal_id;
      while (came_from.count(u)) { path_ids.push_back(u); u = came_from.at(u); }
      path_ids.push_back(start_id);
      std::reverse(path_ids.begin(), path_ids.end());
      break;
    }
    if (gscore.count(cur.id) && cur.g > gscore.at(cur.id)) continue;
    expanded_set.insert(cur.id);
    auto it_adj = local_adj.find(cur.id);
    if (it_adj == local_adj.end()) continue;

    for (const auto& nb : it_adj->second) {
      auto nit = local_nodes.find(nb);
      if (nit == local_nodes.end()) continue;
      if (!nit->second.is_walkable) { ++rejected_not_walkable; continue; }
      if (!hasVerticalClearance(nit->second.center, nit->second.size))
        { ++rejected_vertical_clearance; continue; }
      if (enable_radial_clearance_ && !hasRadialClearanceAbove(nit->second.center, nit->second.size))
        { ++rejected_radial_clearance; continue; }
      if (enable_edge_collision_check_) {
        auto cit = local_nodes.find(cur.id);
        if (cit != local_nodes.end() &&
            !isEdgeCollisionFree(cit->second.center, cit->second.size,
                                 nit->second.center, nit->second.size))
          { ++rejected_edge_collision; continue; }
      }
      ++accepted_neighbors;
      double move_cost = local_nodes.at(cur.id).center.distance(nit->second.center);
      double penalty   = getPrecomputedPenalty(nb);
      double tentative = cur.g + move_cost + penalty;
      if (!gscore.count(nb) || tentative < gscore[nb]) {
        came_from[nb] = cur.id;
        gscore[nb]    = tentative;
        openq.push({nb, tentative + heuristic(nb), tentative});
      }
    }
  }
  auto t_astar_end = std::chrono::steady_clock::now();
  double dur_astar = std::chrono::duration_cast<std::chrono::duration<double>>(
    t_astar_end - t_astar_start).count();
  RCLCPP_INFO(node_->get_logger(),
    "A* took %.3f s, expanded=%zu | accepted=%zu, rejected: walkable=%zu vc=%zu rc=%zu ec=%zu",
    dur_astar, expanded_set.size(),
    accepted_neighbors, rejected_not_walkable,
    rejected_vertical_clearance, rejected_radial_clearance, rejected_edge_collision);

  if (path_ids.empty()) {
    RCLCPP_WARN(node_->get_logger(),
      "NO PATH FOUND: start='%s' goal='%s' expanded=%zu",
      start_id.c_str(), goal_id.c_str(), expanded_set.size());
    if (rejected_vertical_clearance + rejected_radial_clearance + rejected_edge_collision > 0) {
      RCLCPP_WARN(node_->get_logger(),
        "  -> Collision checks blocked %zu + %zu + %zu edges. "
        "Adjust robot_height (%.2f), robot_radius (%.2f), or check octomap for false occupancy.",
        rejected_vertical_clearance, rejected_radial_clearance, rejected_edge_collision,
        robot_height_, robot_radius_);
    }

    // Publish expanded dead-end nodes as yellow markers
    if (!expanded_set.empty()) {
      visualization_msgs::msg::MarkerArray debug_ma;
      visualization_msgs::msg::Marker ym;
      ym.header.frame_id = map_frame.empty() ? "map" : map_frame;
      ym.header.stamp    = now;
      ym.ns              = "failed_expanded";
      ym.id              = 0;
      ym.type            = visualization_msgs::msg::Marker::CUBE_LIST;
      ym.action          = visualization_msgs::msg::Marker::ADD;
      double ys = active_voxel_size > 0.0 ? std::max(active_voxel_size * 1.2, 0.06) : 0.06;
      ym.scale.x = ys; ym.scale.y = ys; ym.scale.z = ys;
      ym.color.r = 1.0f; ym.color.g = 1.0f; ym.color.b = 0.0f; ym.color.a = 1.0f;
      for (const auto& nid : expanded_set) {
        auto it = local_nodes.find(nid);
        if (it == local_nodes.end()) continue;
        geometry_msgs::msg::Point pt;
        pt.x = it->second.center.x();
        pt.y = it->second.center.y();
        pt.z = it->second.center.z();
        ym.points.push_back(pt);
      }
      debug_ma.markers.push_back(ym);
      mapper_->publishAdditionalMarkers(debug_ma);
    }

    message = "No path found on graph";
    return mbf_msgs::action::GetPath::Result::NO_PATH_FOUND;
  }

  // ---- Build plan from path ids ----
  RCLCPP_INFO(node_->get_logger(), "Path found: %zu nodes", path_ids.size());
  for (const auto& id : path_ids) {
    const auto& gn = local_nodes.at(id);
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id   = path_frame;
    p.header.stamp      = now;
    p.pose.position.x   = gn.center.x();
    p.pose.position.y   = gn.center.y();
    p.pose.position.z   = gn.center.z();
    p.pose.orientation.w = 1.0;
    plan.push_back(p);
  }
  // Propagate the navigation goal's orientation to the last waypoint so the controller
  // can enforce heading alignment at the goal. All other waypoints keep identity (w=1).
  if (!plan.empty()) {
    plan.back().pose.orientation = goal.pose.orientation;
  }

  // Publish start/goal markers
  {
    visualization_msgs::msg::MarkerArray ma2;
    visualization_msgs::msg::Marker sgm;
    sgm.header.frame_id = map_frame.empty() ? "map" : map_frame;
    sgm.header.stamp    = now;
    sgm.ns              = "graph_start_goal";
    sgm.id              = 2;
    sgm.type            = visualization_msgs::msg::Marker::CUBE_LIST;
    sgm.action          = visualization_msgs::msg::Marker::ADD;
    double sscale = active_voxel_size > 0.0 ? std::max(active_voxel_size, 0.05) : 0.05;
    sgm.scale.x = static_cast<float>(sscale);
    sgm.scale.y = static_cast<float>(sscale);
    sgm.scale.z = static_cast<float>(sscale);
    sgm.color.r = 1.0f; sgm.color.g = 1.0f; sgm.color.b = 0.0f; sgm.color.a = 0.95f;
    if (!start_id.empty()) {
      const auto& sn = local_nodes.at(start_id);
      geometry_msgs::msg::Point ps;
      ps.x = sn.center.x(); ps.y = sn.center.y(); ps.z = sn.center.z();
      sgm.points.push_back(ps);
    }
    if (!goal_id.empty()) {
      const auto& gn = local_nodes.at(goal_id);
      geometry_msgs::msg::Point pg;
      pg.x = gn.center.x(); pg.y = gn.center.y(); pg.z = gn.center.z();
      sgm.points.push_back(pg);
    }
    if (!sgm.points.empty()) {
      ma2.markers.push_back(sgm);
      mapper_->publishAdditionalMarkers(ma2);
    }
  }

  // ---- Cost + path publish ----
  cost = static_cast<double>(plan.size());

  for (const auto& p : plan) {
    if (!std::isfinite(p.pose.position.x) ||
        !std::isfinite(p.pose.position.y) ||
        !std::isfinite(p.pose.position.z)) {
      RCLCPP_ERROR(node_->get_logger(), "Path contains non-finite coordinates; aborting.");
      return mbf_msgs::action::GetPath::Result::FAILURE;
    }
  }

  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp    = now;
  path_msg.header.frame_id = map_frame.empty() ? start.header.frame_id : map_frame;
  path_msg.poses           = plan;
  path_pub_->publish(path_msg);

  nav_msgs::msg::Path lifted = path_msg;
  for (auto& p : lifted.poses) { p.pose.position.z += 0.5; }
  body_height_path_pub_->publish(lifted);

  RCLCPP_INFO(node_->get_logger(), "Path published: %zu waypoints.", plan.size());
  return mbf_msgs::action::GetPath::Result::SUCCESS;
}

// ---------------------------------------------------------------------------
// findClosestGraphNode
// ---------------------------------------------------------------------------
std::string AstarOctoPlanner::findClosestGraphNode(
  const octomap::point3d& p,
  const std::shared_ptr<mbf_octo_core::GraphData>& graph) const
{
  if (!graph || graph->nodes.empty()) return {};
  const double xy_search_radius = 1.0;
  double best = std::numeric_limits<double>::infinity();
  std::string best_id;

  for (const auto& kv : graph->nodes) {
    if (!kv.second.is_walkable) continue;
    double dx = kv.second.center.x() - p.x();
    double dy = kv.second.center.y() - p.y();
    double dxy_sq = dx*dx + dy*dy;
    if (dxy_sq > xy_search_radius * xy_search_radius) continue;
    double node_z = kv.second.center.z();
    if (node_z > p.z() + max_z_above_query_) continue;
    double dz    = std::abs(node_z - p.z());
    double score = dz + 0.1 * std::sqrt(dxy_sq);
    if (score < best) { best = score; best_id = kv.first; }
  }

  if (best_id.empty()) {
    // Fallback: unconstrained 3D distance
    best = std::numeric_limits<double>::infinity();
    for (const auto& kv : graph->nodes) {
      if (!kv.second.is_walkable) continue;
      double dx = kv.second.center.x() - p.x();
      double dy = kv.second.center.y() - p.y();
      double dz = kv.second.center.z() - p.z();
      double d  = dx*dx + dy*dy + dz*dz;
      if (d < best) { best = d; best_id = kv.first; }
    }
  }
  return best_id;
}

// ---------------------------------------------------------------------------
// planOnGraph (graph-parameterized A* — for external / test use)
// ---------------------------------------------------------------------------
std::vector<std::string> AstarOctoPlanner::planOnGraph(
  const std::string& start_id, const std::string& goal_id,
  const std::shared_ptr<mbf_octo_core::GraphData>& graph) const
{
  if (!graph || start_id.empty() || goal_id.empty()) return {};
  if (!graph->nodes.count(start_id) || !graph->nodes.count(goal_id)) return {};

  struct Item { std::string id; double f; double g; };
  struct Cmp  { bool operator()(const Item& a, const Item& b) const { return a.f > b.f; } };
  std::priority_queue<Item, std::vector<Item>, Cmp> open;
  std::unordered_map<std::string, double> gscore;
  std::unordered_map<std::string, std::string> came_from;

  auto heuristic = [&](const std::string& a) -> double {
    const auto& pa = graph->nodes.at(a).center;
    const auto& pg = graph->nodes.at(goal_id).center;
    double dx = pa.x()-pg.x(); double dy = pa.y()-pg.y(); double dz = pa.z()-pg.z();
    return std::sqrt(dx*dx+dy*dy+dz*dz);
  };

  open.push({start_id, heuristic(start_id), 0.0});
  gscore[start_id] = 0.0;

  while (!open.empty()) {
    Item cur = open.top(); open.pop();
    if (cur.id == goal_id) {
      std::vector<std::string> path;
      std::string u = goal_id;
      while (came_from.count(u)) { path.push_back(u); u = came_from.at(u); }
      path.push_back(start_id);
      std::reverse(path.begin(), path.end());
      return path;
    }
    if (gscore.count(cur.id) && cur.g > gscore.at(cur.id)) continue;
    auto it = graph->adj.find(cur.id);
    if (it == graph->adj.end()) continue;
    for (const auto& nb : it->second) {
      if (!graph->nodes.count(cur.id) || !graph->nodes.count(nb)) continue;
      double tentative = cur.g + graph->nodes.at(cur.id).center.distance(graph->nodes.at(nb).center);
      if (!gscore.count(nb) || tentative < gscore[nb]) {
        came_from[nb] = cur.id;
        gscore[nb]    = tentative;
        open.push({nb, tentative + heuristic(nb), tentative});
      }
    }
  }
  return {};
}

// ---------------------------------------------------------------------------
// gridToWorld (uses mapper for bounds + voxel size)
// ---------------------------------------------------------------------------
std::array<double, 3> AstarOctoPlanner::gridToWorld(const std::tuple<int, int, int>& grid_pt)
{
  std::array<double, 3> min_b, max_b;
  mapper_->getBounds(min_b, max_b);
  const double vx = mapper_->getVoxelSize();
  int ix, iy, iz;
  std::tie(ix, iy, iz) = grid_pt;
  return {
    min_b[0] + static_cast<double>(ix) * vx,
    min_b[1] + static_cast<double>(iy) * vx,
    min_b[2] + static_cast<double>(iz) * vx
  };
}

// ---------------------------------------------------------------------------
// isCylinderCollisionFree (uses mapper octree / voxel size)
// ---------------------------------------------------------------------------
bool AstarOctoPlanner::isCylinderCollisionFree(
  const std::tuple<int, int, int>& coord, double /*radius*/)
{
  if (!mapper_) return true;
  const octomap::OcTree* octree = mapper_->getOctree();
  if (!octree) return true;
  const double voxel = mapper_->getVoxelSize();

  auto w = gridToWorld(coord);
  double half_w = 0.5 * (robot_width_  + footprint_margin_);
  double half_l = 0.5 * (robot_length_ + footprint_margin_);
  const int nx = std::max(1, footprint_samples_x_);
  const int ny = std::max(1, footprint_samples_y_);
  double z_bottom = w[2];
  double z_top    = w[2] + robot_height_;
  const int nz = std::max(1, static_cast<int>(std::ceil(robot_height_ / std::max(0.05, voxel))));

  for (int iz = 0; iz < nz; ++iz) {
    double frac = nz == 1 ? 0.5 : static_cast<double>(iz) / static_cast<double>(nz - 1);
    double z = z_bottom + frac * (z_top - z_bottom);
    for (int ix = 0; ix < nx; ++ix) {
      double fx = nx == 1 ? 0.0 : (static_cast<double>(ix) / static_cast<double>(nx-1) * 2.0 - 1.0);
      double sx = w[0] + fx * half_l;
      for (int iy = 0; iy < ny; ++iy) {
        double fy = ny == 1 ? 0.0 : (static_cast<double>(iy) / static_cast<double>(ny-1) * 2.0 - 1.0);
        double sy = w[1] + fy * half_w;
        octomap::OcTreeNode* n = octree->search(sx, sy, z);
        if (n && octree->isNodeOccupied(n)) return false;
      }
    }
  }
  // Also check centre column
  for (double z = z_bottom; z <= z_top; z += std::max(voxel, 0.05)) {
    octomap::OcTreeNode* c = octree->search(w[0], w[1], z);
    if (c && octree->isNodeOccupied(c)) return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// clearOccupiedCylinderAround
// ---------------------------------------------------------------------------
void AstarOctoPlanner::clearOccupiedCylinderAround(
  const geometry_msgs::msg::Point& center,
  double radius, double z_bottom, double z_top)
{
  if (!mapper_) return;
  // We need a mutable octree here; get the non-const pointer via the octree
  // The mapper's octree is exposed as const. Temporarily cast for clearing.
  const octomap::OcTree* octree_const = mapper_->getOctree();
  if (!octree_const) return;
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
  octomap::OcTree* octree = const_cast<octomap::OcTree*>(octree_const);

  const double step = std::max(mapper_->getVoxelSize(), 0.05);
  int n = static_cast<int>(std::ceil((2.0 * radius) / step));
  for (int ix = -n; ix <= n; ++ix) {
    for (int iy = -n; iy <= n; ++iy) {
      double x = center.x + ix * step;
      double y = center.y + iy * step;
      double dx = x - center.x, dy = y - center.y;
      if ((dx*dx + dy*dy) > (radius+step)*(radius+step)) continue;
      for (double z = z_bottom; z <= z_top; z += step) {
        octomap::OcTreeNode* nd = octree->search(x, y, z);
        if (nd && octree->isNodeOccupied(nd))
          octree->updateNode(octomap::point3d(x, y, z), false);
      }
    }
  }
}

// ---------------------------------------------------------------------------
// hasVerticalClearance  (2-arg — live A* check using mapper)
// ---------------------------------------------------------------------------
bool AstarOctoPlanner::hasVerticalClearance(
  const octomap::point3d& center, double node_size) const
{
  if (!mapper_) return true;
  const octomap::OcTree* octree = mapper_->getOctree();
  if (!octree) return true;
  if (robot_height_ <= 0.0) return true;

  const double voxel          = mapper_->getVoxelSize();
  const double climbable      = std::max(0.0, mapper_->getMaxStepHeight());
  const double stepz          = std::max(voxel, 0.05);
  const double voxel_top      = center.z() + 0.5 * node_size;
  const double z_start        = voxel_top + climbable + 0.01;
  const double z_end          = voxel_top + robot_height_;

  if (z_start > z_end + 1e-6) return true;

  for (double z = z_start; z <= z_end + 1e-6; z += stepz) {
    octomap::OcTreeNode* nn = octree->search(center.x(), center.y(), z);
    if (nn && octree->isNodeOccupied(nn)) {
      // A lone top-surface voxel (nothing above it) is walkable terrain, not a ceiling.
      octomap::OcTreeNode* above = octree->search(center.x(), center.y(), z + voxel);
      if (above && octree->isNodeOccupied(above)) return false;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// hasRadialClearanceAbove  (live A* check)
// ---------------------------------------------------------------------------
bool AstarOctoPlanner::hasRadialClearanceAbove(
  const octomap::point3d& center, double node_size) const
{
  if (!mapper_) return true;
  const octomap::OcTree* octree = mapper_->getOctree();
  if (!octree) return true;
  if (robot_radius_ <= 0.0 || robot_height_ <= 0.0) return true;

  const double voxel     = mapper_->getVoxelSize();
  const double climbable = std::max(0.0, mapper_->getMaxStepHeight());
  const double voxel_top = center.z() + 0.5 * node_size;
  const double z_min     = voxel_top + climbable + 0.02;
  const double z_max     = voxel_top + robot_height_;
  if (z_min > z_max + 1e-6) return true;

  const int    num_dirs   = std::max(4, radial_clearance_num_dirs_);
  const int    num_z      = std::max(1, radial_clearance_num_z_);
  const double angle_step = 2.0 * M_PI / static_cast<double>(num_dirs);

  for (int iz = 0; iz < num_z; ++iz) {
    double frac = num_z == 1 ? 0.5 : static_cast<double>(iz) / static_cast<double>(num_z-1);
    double z    = z_min + frac * (z_max - z_min);
    for (int idir = 0; idir < num_dirs; ++idir) {
      double angle = idir * angle_step;
      double sx = center.x() + robot_radius_ * std::cos(angle);
      double sy = center.y() + robot_radius_ * std::sin(angle);
      octomap::OcTreeNode* nn = octree->search(sx, sy, z);
      if (nn && octree->isNodeOccupied(nn)) {
        // Distinguish wall interior (has occupancy above → real obstacle) from
        // walkable top surface (no occupancy above → terrain, not an obstacle).
        octomap::OcTreeNode* above = octree->search(sx, sy, z + voxel);
        if (above && octree->isNodeOccupied(above)) return false;
      }
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// isEdgeCollisionFree  (live A* check)
// ---------------------------------------------------------------------------
bool AstarOctoPlanner::isEdgeCollisionFree(
  const octomap::point3d& from, double from_size,
  const octomap::point3d& to,   double to_size) const
{
  if (!mapper_) return true;
  const octomap::OcTree* octree = mapper_->getOctree();
  if (!octree) return true;
  if (robot_height_ <= 0.0) return true;

  const double dist = from.distance(to);
  if (dist < 1e-6) return true;

  const double voxel     = mapper_->getVoxelSize();
  const double climbable = std::max(0.0, mapper_->getMaxStepHeight());
  const double step      = std::max(voxel * 0.5, 0.05);
  const double z_step    = std::max(voxel, 0.1);
  const int    n_samples = std::max(2, static_cast<int>(std::ceil(dist / step)));

  const double from_top = from.z() + 0.5 * from_size;
  const double to_top   = to.z()   + 0.5 * to_size;

  for (int i = 1; i < n_samples; ++i) {
    double t          = static_cast<double>(i) / static_cast<double>(n_samples);
    double sx         = from.x() + t * (to.x() - from.x());
    double sy         = from.y() + t * (to.y() - from.y());
    double surface_z  = from_top + t * (to_top - from_top);

    // The linear interpolation of surface_z can land inside stair/ramp solid when
    // the terrain rises faster than the linear ramp (e.g. ascending stairs).  Scan
    // upward by at most climbable to find the actual surface top.
    // IMPORTANT: the scan cap must be frozen before the loop.  If surface_z were
    // used in the loop condition it would grow with each iteration, letting the scan
    // climb indefinitely through walls and then check above them (empty) — a false
    // pass.  With a fixed cap, a wall taller than climbable is left partially inside
    // the adjusted check band and gets detected normally below.
    {
      const double scan_cap = surface_z + climbable;
      for (double sz = surface_z; sz < scan_cap; sz += voxel) {
        octomap::OcTreeNode* tn = octree->search(sx, sy, sz);
        if (tn && octree->isNodeOccupied(tn)) surface_z = sz + voxel;
        else break;
      }
    }

    const double z_start = surface_z + climbable + 0.01;
    const double z_end   = surface_z + robot_height_;
    if (z_start > z_end + 1e-6) continue;

    const double body_band = z_end - z_start;
    const int    n_z       = std::max(1, static_cast<int>(std::ceil(body_band / z_step)));
    for (int iz = 0; iz <= n_z; ++iz) {
      double frac    = n_z == 0 ? 0.0 : static_cast<double>(iz) / static_cast<double>(n_z);
      double check_z = z_start + frac * body_band;
      octomap::OcTreeNode* nn = octree->search(sx, sy, check_z);
      if (nn && octree->isNodeOccupied(nn)) {
        // Only block if this is a wall interior (has occupancy one voxel higher).
        // A top-surface voxel (walkable terrain) has nothing above it and should
        // not count as a collision for the robot body check.
        octomap::OcTreeNode* above = octree->search(sx, sy, check_z + voxel);
        if (above && octree->isNodeOccupied(above)) return false;
      }
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// reconfigureCallback  (planner-specific params only)
// ---------------------------------------------------------------------------
rcl_interfaces::msg::SetParametersResult
AstarOctoPlanner::reconfigureCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto& p : parameters) {
    const std::string& n = p.get_name();
    if      (n == name_ + ".cost_limit")              { config_.cost_limit              = p.as_double(); }
    else if (n == name_ + ".robot_radius")            { robot_radius_                   = p.as_double(); }
    else if (n == name_ + ".robot_width")             { robot_width_                    = p.as_double(); }
    else if (n == name_ + ".robot_length")            { robot_length_                   = p.as_double(); }
    else if (n == name_ + ".robot_height")            {
      robot_height_ = p.as_double();
      // height change affects what getReadyGraph() revalidates; mark mapper dirty
      if (mapper_) mapper_->markPenaltiesDirty();
    }
    else if (n == name_ + ".footprint_margin")        { footprint_margin_               = p.as_double(); }
    else if (n == name_ + ".footprint_samples_x")     { footprint_samples_x_            = p.as_int(); }
    else if (n == name_ + ".footprint_samples_y")     { footprint_samples_y_            = p.as_int(); }
    else if (n == name_ + ".min_vertical_clearance")  { min_vertical_clearance_         = p.as_double(); }
    else if (n == name_ + ".max_vertical_clearance")  { max_vertical_clearance_         = p.as_double(); }
    else if (n == name_ + ".radial_clearance_num_dirs") { radial_clearance_num_dirs_    = p.as_int(); }
    else if (n == name_ + ".radial_clearance_num_z")    { radial_clearance_num_z_       = p.as_int(); }
    else if (n == name_ + ".enable_radial_clearance")   { enable_radial_clearance_      = p.as_bool(); }
    else if (n == name_ + ".enable_edge_collision_check") { enable_edge_collision_check_ = p.as_bool(); }
    else if (n == name_ + ".max_z_above_query")         { max_z_above_query_            = p.as_double(); }
    RCLCPP_INFO(node_->get_logger(), "Updated planner param: %s", n.c_str());
  }
  result.successful = true;
  return result;
}

}  // namespace astar_octo_planner
