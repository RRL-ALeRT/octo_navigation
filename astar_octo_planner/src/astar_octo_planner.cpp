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
#include <octomap_msgs/conversions.h>

#include <mbf_msgs/action/get_path.hpp>
#include <astar_octo_planner/astar_octo_planner.h>

#include <queue>
#include <algorithm>
#include <random>
#include <cmath>
#include <thread>
#include <atomic>
#include <chrono>

PLUGINLIB_EXPORT_CLASS(astar_octo_planner::AstarOctoPlanner, mbf_octo_core::OctoPlanner);

namespace astar_octo_planner
{

// Legacy grid-based A* structures removed; planner uses graph-based A* (planOnGraph)

AstarOctoPlanner::AstarOctoPlanner()
: voxel_size_(0.1),  // double than that of octomap resolution
  z_threshold_(0.3)   // same as Z_THRESHOLD in Python
{
  // Planner now relies on octomap as the authoritative map source.
  // Set a default minimum bound; this will be updated when processing octomap messages.
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

  // Transform start/goal into the octomap frame (map_frame_) if necessary.
  geometry_msgs::msg::PoseStamped start_in_map = start;
  geometry_msgs::msg::PoseStamped goal_in_map = goal;

  // This planner no longer performs TF transforms. The start and goal are
  // used exactly as provided (no frame checks or automatic transforms).

  // We'll compute grid indices after we set the active voxel size (from octree).
  // Leave start_in_map and goal_in_map as the provided poses for now.
  // Planner now requires an octree as the authoritative map source. If we don't
  // have one yet, return failure instead of falling back to a dense occupancy
  // grid (which has been removed to avoid large allocations).
  if (!octree_) {
    RCLCPP_WARN(node_->get_logger(), "No octomap available; cannot plan. Waiting for %s", octomap_topic_.c_str());
    return mbf_msgs::action::GetPath::Result::FAILURE;
  }

  // Use octree resolution as the active voxel size to avoid mismatches
  active_voxel_size_ = std::max(voxel_size_, octree_->getResolution());

  // Use the provided start as the robot's true starting position (don't snap it).
  // Snap only the goal to the nearest occupied leaf so the planner aims at surface points.
  // RCLCPP_INFO(node_->get_logger(), "Using provided start: x=%.3f y=%.3f z=%.3f (frame=%s)",
  //             start_in_map.pose.position.x, start_in_map.pose.position.y, start_in_map.pose.position.z,
  //             start_in_map.header.frame_id.c_str());
  // Snap start/goal world coordinates to the active voxel size to avoid tiny
  // floating-point differences and reduce search effort (e.g. resolution=0.1 -> one decimal).
  double vs_snap = active_voxel_size_;
    // Clear occupied voxels around the robot start pose to avoid leg points
    // blocking the initial planning steps. This modifies the in-memory octree
    // only and does not publish or persist changes back to other systems.
  auto snap_world = [vs_snap](geometry_msgs::msg::Point &p) {
    p.x = std::round(p.x / vs_snap) * vs_snap;
    p.y = std::round(p.y / vs_snap) * vs_snap;
    p.z = std::round(p.z / vs_snap) * vs_snap;
  };
  // Create mutable copies for snapping
  geometry_msgs::msg::Point start_world = start_in_map.pose.position;
  geometry_msgs::msg::Point goal_world = goal_in_map.pose.position;
  snap_world(start_world);
  snap_world(goal_world);
  RCLCPP_INFO(node_->get_logger(), "Snapped start to: x=%.3f y=%.3f z=%.3f",
              start_world.x, start_world.y, start_world.z);

  plan.clear();
  std::string path_frame = map_frame_.empty() ? start.header.frame_id : map_frame_;
  rclcpp::Time now = node_->now();
    if (octree_) {
      double clear_radius = robot_radius_ + footprint_margin_;
      double z_bottom = start_in_map.pose.position.z - 0.1; // a little below base
      double z_top = start_in_map.pose.position.z + robot_height_ + 0.1;
      clearOccupiedCylinderAround(start_in_map.pose.position, clear_radius, z_bottom, z_top);
      RCLCPP_INFO(node_->get_logger(), "Cleared occupied voxels around start pose within radius %.3f m and z [%.3f..%.3f]", clear_radius, z_bottom, z_top);
    }
    // Try to use the precomputed connectivity graph if available. If graph is
    // missing or yields no route, fall back to a simple two-point plan.
    {
      // Ensure graph is built if dirty or empty (build synchronously here).
      std::lock_guard<std::mutex> lock(graph_mutex_);
      if ((graph_nodes_.empty() || graph_dirty_)) {
        RCLCPP_INFO(node_->get_logger(), "Graph missing or dirty — building connectivity graph synchronously...");
        buildConnectivityGraph();
        graph_dirty_ = false;
        RCLCPP_INFO(node_->get_logger(), "Graph built: nodes=%zu", graph_nodes_.size());
      }
    }

    // Find closest graph nodes to start and goal
    std::string start_id;
    std::string goal_id;
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      start_id = findClosestGraphNode(octomap::point3d(start_world.x, start_world.y, start_world.z));
      goal_id  = findClosestGraphNode(octomap::point3d(goal_world.x,  goal_world.y,  goal_world.z));
    }

    RCLCPP_INFO(node_->get_logger(), "Graph attempt: start_id='%s' goal_id='%s'",
                start_id.empty() ? "<none>" : start_id.c_str(),
                goal_id.empty() ? "<none>" : goal_id.c_str());

    auto valid_center = [&](const octomap::point3d &c)->bool {
      if (!std::isfinite(c.x()) || !std::isfinite(c.y()) || !std::isfinite(c.z())) return false;
      if (c.x() < min_bound_[0] - 1e-6 || c.x() > max_bound_[0] + 1e-6) return false;
      if (c.y() < min_bound_[1] - 1e-6 || c.y() > max_bound_[1] + 1e-6) return false;
      if (c.z() < min_bound_[2] - 1e-6 || c.z() > max_bound_[2] + 1e-6) return false;
      return true;
    };

    bool centers_ok = true;
    if (!start_id.empty()) {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      const auto &s = graph_nodes_.at(start_id);
      RCLCPP_INFO(node_->get_logger(), "Start graph center: (%.3f, %.3f, %.3f)", s.center.x(), s.center.y(), s.center.z());
      centers_ok &= valid_center(s.center);
    }
    if (!goal_id.empty()) {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      const auto &g = graph_nodes_.at(goal_id);
      RCLCPP_INFO(node_->get_logger(), "Goal graph center: (%.3f, %.3f, %.3f)", g.center.x(), g.center.y(), g.center.z());
      centers_ok &= valid_center(g.center);
    }

    if (!centers_ok || start_id.empty() || goal_id.empty()) {
      RCLCPP_WARN(node_->get_logger(), "Graph path not available or invalid centers. Aborting plan (NO_PATH_FOUND) for debugging.");
      message = "Graph path not available or invalid centers";
      return mbf_msgs::action::GetPath::Result::NO_PATH_FOUND;
    } else {
      // Compute path on graph with lazy validation (per-plan caches).
      // Copy graph data under lock so we can run expensive checks without holding the mutex.
      std::unordered_map<std::string, GraphNode> local_nodes;
      std::unordered_map<std::string, std::vector<std::string>> local_adj;
      std::unordered_map<std::string, double> local_node_penalty;
      std::unordered_set<std::string> local_penalized_nodes;
      {
        std::lock_guard<std::mutex> lock(graph_mutex_);
        local_nodes = graph_nodes_;
        local_adj = graph_adj_;
        // copy precomputed penalties
        local_node_penalty = graph_node_penalty_;
        local_penalized_nodes = graph_penalized_nodes_;
      }

          // Prepare per-plan containers for search and visualization.
          std::unordered_set<std::string> expanded_set;
          std::unordered_map<std::string, double> gscore;
          std::unordered_map<std::string, std::string> came_from;
          struct PQItem { std::string id; double f; double g; };
          struct PQCmp { bool operator()(const PQItem &a, const PQItem &b) const { return a.f > b.f; } };
          std::priority_queue<PQItem, std::vector<PQItem>, PQCmp> openq;

          auto heuristic_local = [&](const std::string &a)->double{
            if (local_nodes.find(a) == local_nodes.end() || local_nodes.find(goal_id) == local_nodes.end()) return 0.0;
            const auto &pa = local_nodes.at(a).center;
            const auto &pg = local_nodes.at(goal_id).center;
            double dx = pa.x()-pg.x(); double dy = pa.y()-pg.y(); double dz = pa.z()-pg.z();
            return std::sqrt(dx*dx + dy*dy + dz*dz);
          };

          // keep copies using the exact names other code expects for marker publishing
          std::unordered_map<std::string,double> node_penalty = local_node_penalty;
          std::unordered_set<std::string> penalized_nodes = local_penalized_nodes;

          auto getPrecomputedPenalty = [&](const std::string &nid)->double{
            auto it = local_node_penalty.find(nid);
            if (it != local_node_penalty.end()) return it->second;
            return 0.0;
          };

          // initialize search
          openq.push({start_id, heuristic_local(start_id), 0.0});
          gscore[start_id] = 0.0;

          std::vector<std::string> path_ids;
          auto t_astar_start = std::chrono::steady_clock::now();
          // A* loop
          while (!openq.empty()) {
            PQItem cur = openq.top(); openq.pop();
            if (cur.id == goal_id) {
              // reconstruct path
              std::string u = goal_id;
              while (came_from.find(u) != came_from.end()) { path_ids.push_back(u); u = came_from.at(u); }
              path_ids.push_back(start_id);
              std::reverse(path_ids.begin(), path_ids.end());
              break;
            }
            if (gscore.find(cur.id) != gscore.end() && cur.g > gscore.at(cur.id)) continue; // stale
            expanded_set.insert(cur.id);
            auto it_adj = local_adj.find(cur.id);
            if (it_adj == local_adj.end()) continue;
            for (const auto &nb : it_adj->second) {
              if (local_nodes.find(nb) == local_nodes.end()) continue;
              double move_cost = local_nodes.at(cur.id).center.distance(local_nodes.at(nb).center);
              double penalty = getPrecomputedPenalty(nb);
              node_penalty[nb] = penalty;
              if (penalty > 1e-9) penalized_nodes.insert(nb);
              double tentative = cur.g + move_cost + penalty;
              if (gscore.find(nb) == gscore.end() || tentative < gscore[nb]) {
                came_from[nb] = cur.id;
                gscore[nb] = tentative;
                openq.push({nb, tentative + heuristic_local(nb), tentative});
              }
            }
          }
          auto t_astar_end = std::chrono::steady_clock::now();
          double dur_astar = std::chrono::duration_cast<std::chrono::duration<double>>(t_astar_end - t_astar_start).count();
          RCLCPP_INFO(node_->get_logger(), "A* search took %.3f s, expanded=%zu nodes", dur_astar, expanded_set.size());

      if (path_ids.empty()) {
        RCLCPP_WARN(node_->get_logger(), "No path found on graph between %s and %s — aborting (NO_PATH_FOUND)", start_id.c_str(), goal_id.c_str());
        message = "No path found on graph";
        return mbf_msgs::action::GetPath::Result::NO_PATH_FOUND;
      }

      // Convert found path ids to PoseStamped
      RCLCPP_INFO(node_->get_logger(), "Graph path ids (%zu):", path_ids.size());
      for (const auto &id : path_ids) {
        RCLCPP_INFO(node_->get_logger(), "  -> %s", id.c_str());
        const auto &gn = local_nodes.at(id);
        geometry_msgs::msg::PoseStamped p;
        p.header.frame_id = path_frame;
        p.header.stamp = now;
        p.pose.position.x = gn.center.x();
        p.pose.position.y = gn.center.y();
        p.pose.position.z = gn.center.z();
        p.pose.orientation.w = 1.0;
        plan.push_back(p);
      }

      // Publish debug markers for expanded nodes that did not end up on the final path
      if (publish_graph_markers_ && graph_marker_pub_) {
        // build a set of path node ids for quick lookup
        std::unordered_set<std::string> path_set(path_ids.begin(), path_ids.end());
        visualization_msgs::msg::Marker dead_mark;
  dead_mark.header.frame_id = map_frame_.empty() ? "map" : map_frame_;
        dead_mark.header.stamp = node_->now();
        dead_mark.ns = "graph_dead_nodes";
        dead_mark.id = 1;
        dead_mark.type = visualization_msgs::msg::Marker::CUBE_LIST;
        dead_mark.action = visualization_msgs::msg::Marker::ADD;
        double scale = active_voxel_size_ > 0.0 ? std::max(active_voxel_size_, 0.05) : 0.05;
        dead_mark.scale.x = static_cast<float>(scale);
        dead_mark.scale.y = static_cast<float>(scale);
        dead_mark.scale.z = static_cast<float>(scale);
        // red
        dead_mark.color.r = 1.0f;
        dead_mark.color.g = 0.0f;
        dead_mark.color.b = 0.0f;
        dead_mark.color.a = 0.85f;
        for (const auto &nid : expanded_set) {
          if (path_set.find(nid) != path_set.end()) continue;
          // only include expanded nodes that were penalized (i.e. corner/edge)
          if (penalized_nodes.find(nid) == penalized_nodes.end()) continue;
          auto itn = local_nodes.find(nid);
          if (itn == local_nodes.end()) continue;
          geometry_msgs::msg::Point p;
          p.x = itn->second.center.x();
          p.y = itn->second.center.y();
          p.z = itn->second.center.z();
          dead_mark.points.push_back(p);
        }
  visualization_msgs::msg::MarkerArray ma;
  ma.markers.push_back(dead_mark);
  RCLCPP_INFO(node_->get_logger(), "Publishing dead-node markers: points=%zu", dead_mark.points.size());
  graph_marker_pub_->publish(ma);
      }

      // Also publish start/goal markers (yellow) if temporary nodes exist
      if (publish_graph_markers_ && graph_marker_pub_) {
        visualization_msgs::msg::Marker sgm;
  sgm.header.frame_id = map_frame_.empty() ? "map" : map_frame_;
        sgm.header.stamp = node_->now();
        sgm.ns = "graph_start_goal";
        sgm.id = 2;
        sgm.type = visualization_msgs::msg::Marker::CUBE_LIST;
        sgm.action = visualization_msgs::msg::Marker::ADD;
        double sscale = active_voxel_size_ > 0.0 ? std::max(active_voxel_size_, 0.05) : 0.05;
        sgm.scale.x = static_cast<float>(sscale);
        sgm.scale.y = static_cast<float>(sscale);
        sgm.scale.z = static_cast<float>(sscale);
        // yellow
        sgm.color.r = 1.0f;
        sgm.color.g = 1.0f;
        sgm.color.b = 0.0f;
        sgm.color.a = 0.95f;
        // add start and goal points if present
        auto it_s = local_nodes.find(std::string("__tmp_start"));
        if (it_s != local_nodes.end()) {
          geometry_msgs::msg::Point ps;
          ps.x = it_s->second.center.x(); ps.y = it_s->second.center.y(); ps.z = it_s->second.center.z();
          sgm.points.push_back(ps);
        }
        auto it_g = local_nodes.find(std::string("__tmp_goal"));
        if (it_g != local_nodes.end()) {
          geometry_msgs::msg::Point pg;
          pg.x = it_g->second.center.x(); pg.y = it_g->second.center.y(); pg.z = it_g->second.center.z();
          sgm.points.push_back(pg);
        }
        if (!sgm.points.empty()) {
          visualization_msgs::msg::MarkerArray ma2; ma2.markers.push_back(sgm);
          RCLCPP_INFO(node_->get_logger(), "Publishing start/goal markers: points=%zu", sgm.points.size());
          graph_marker_pub_->publish(ma2);
        }
      }

      // Optionally publish penalty markers (color-coded) for nodes with non-zero penalty
      if (publish_graph_markers_ && graph_marker_pub_) {
        visualization_msgs::msg::Marker pen_m;
        pen_m.header.frame_id = map_frame_.empty() ? "map" : map_frame_;
        pen_m.header.stamp = node_->now();
        pen_m.ns = "graph_penalty";
        pen_m.id = 3;
        pen_m.type = visualization_msgs::msg::Marker::CUBE_LIST;
        pen_m.action = visualization_msgs::msg::Marker::ADD;
        double pscale = active_voxel_size_ > 0.0 ? std::max(active_voxel_size_, 0.05) : 0.05;
        pen_m.scale.x = static_cast<float>(pscale);
        pen_m.scale.y = static_cast<float>(pscale);
        pen_m.scale.z = static_cast<float>(pscale);
        for (const auto &kv : node_penalty) {
          double pen = kv.second;
          if (pen <= 1e-6) continue;
          auto itn = local_nodes.find(kv.first);
          if (itn == local_nodes.end()) continue;
          geometry_msgs::msg::Point p;
          p.x = itn->second.center.x(); p.y = itn->second.center.y(); p.z = itn->second.center.z();
          pen_m.points.push_back(p);
          // color ramp: low green -> high red
          std_msgs::msg::ColorRGBA c;
          double v = std::min(1.0, pen / std::max(1e-6, corner_penalty_weight_));
          c.r = static_cast<float>(v);
          c.g = static_cast<float>(1.0 - v);
          c.b = 0.0f;
          c.a = 0.9f;
          pen_m.colors.push_back(c);
        }
        if (!pen_m.points.empty()) {
          visualization_msgs::msg::MarkerArray pma; pma.markers.push_back(pen_m);
          RCLCPP_INFO(node_->get_logger(), "Publishing penalty markers: points=%zu", pen_m.points.size());
          graph_marker_pub_->publish(pma);
        }
      }
    }

  // Compute a simple cost (e.g., number of steps) – replace with a proper cost calculation if desired.
  cost = static_cast<double>(plan.size());

  // Publish the path for visualization. Before publishing sanity-check values.
  for (const auto &p : plan) {
    if (!std::isfinite(p.pose.position.x) || !std::isfinite(p.pose.position.y) || !std::isfinite(p.pose.position.z)) {
      RCLCPP_ERROR(node_->get_logger(), "Path contains non-finite coordinates; aborting publish.");
      return mbf_msgs::action::GetPath::Result::FAILURE;
    }
    // If coordinates are outside reasonable bounds (e.g. > 1e6), abort and log.
    if (std::fabs(p.pose.position.x) > 1e6 || std::fabs(p.pose.position.y) > 1e6 || std::fabs(p.pose.position.z) > 1e6) {
      RCLCPP_ERROR(node_->get_logger(), "Path contains extremely large coordinates (x=%.3f y=%.3f z=%.3f); aborting.",
                   p.pose.position.x, p.pose.position.y, p.pose.position.z);
      return mbf_msgs::action::GetPath::Result::FAILURE;
    }
  }

  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = node_->now();
  // Publish in the octomap map frame (if set), else fall back to start frame.
  path_msg.header.frame_id = map_frame_.empty() ? start.header.frame_id : map_frame_;
  path_msg.poses = plan;
  path_pub_->publish(path_msg);

  RCLCPP_INFO_STREAM(node_->get_logger(), "Path found with length: " << cost << " steps");

  return mbf_msgs::action::GetPath::Result::SUCCESS;
}

// legacy nearest-3d helpers removed; graph-based planner uses nearest graph nodes


// ------------------ Prototype graph builder / planner ------------------

void AstarOctoPlanner::buildConnectivityGraph(double eps)
{
  auto t_graph_start = std::chrono::steady_clock::now();
  graph_nodes_.clear();
  graph_adj_.clear();
  if (!octree_) return;

  // First pass: collect candidate free leaf nodes (use leaf iterator available in this OctoMap build)
  size_t total_leafs = 0;
  size_t free_leafs = 0;
  size_t skipped_nonfinite = 0;
  size_t skipped_out_of_bounds = 0;
  size_t skipped_collision = 0;
  size_t skipped_no_surface = 0;
  size_t skipped_step_too_high = 0;
  size_t inserted = 0;
  std::atomic<size_t> rejected_distance{0};
  std::atomic<size_t> rejected_los{0};

  //Build surface graph from occupied leafs (top face of occupied voxels)
  for (auto it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it) {
    ++total_leafs;
    if (!octree_->isNodeOccupied(*it)) continue;
    ++free_leafs; // reusing counter for occupied samples here
    octomap::point3d occ = it.getCoordinate();
    double node_z = occ.z() + 0.5 * octree_->getNodeSize(it.getDepth());
    // nudge node slightly above to be in free space
    node_z += std::max(active_voxel_size_, 0.01);
    if (!std::isfinite(occ.x()) || !std::isfinite(occ.y()) || !std::isfinite(node_z)) { ++skipped_nonfinite; continue; }
    if (occ.x() < min_bound_[0] - 1e-6 || occ.x() > max_bound_[0] + 1e-6 ||
        occ.y() < min_bound_[1] - 1e-6 || occ.y() > max_bound_[1] + 1e-6 ||
        node_z < min_bound_[2] - 1e-6 || node_z > max_bound_[2] + 1e-6) { ++skipped_out_of_bounds; continue; }

    // vertical clearance check above surface
    bool vertical_clear = true;
    double stepz = std::max(active_voxel_size_, 0.05);
    for (double z = node_z; z <= node_z + robot_height_; z += stepz) {
      octomap::OcTreeNode* nn = octree_->search(occ.x(), occ.y(), z);
      if (nn && octree_->isNodeOccupied(nn)) { vertical_clear = false; break; }
    }
    if (!vertical_clear) { ++skipped_collision; continue; }

    octomap::point3d center(occ.x(), occ.y(), node_z);
    octomap::OcTreeKey key = octree_->coordToKey(center);
    unsigned int depth = it.getDepth();
    double size = octree_->getNodeSize(depth);

    GraphNode gn;
    gn.key = key;
    gn.depth = depth;
    gn.center = center;
    gn.size = size;

    // create stable id
    char idbuf[128];
    int ix_mm = static_cast<int>(std::round(center.x() * 1000.0));
    int iy_mm = static_cast<int>(std::round(center.y() * 1000.0));
    int iz_mm = static_cast<int>(std::round(center.z() * 1000.0));
    std::snprintf(idbuf, sizeof(idbuf), "c_%d_%d_%d", ix_mm, iy_mm, iz_mm);
    graph_nodes_.emplace(std::string(idbuf), gn);
    ++inserted;
  }

  // RCLCPP_INFO(node_->get_logger(), "Graph build stats: total_leafs=%zu free_leafs=%zu nonfinite=%zu out_of_bounds=%zu inserted=%zu",
  //             total_leafs, free_leafs, skipped_nonfinite, skipped_out_of_bounds, inserted);
  // RCLCPP_INFO(node_->get_logger(), "Graph build extra: skipped_collision=%zu", skipped_collision);
  // RCLCPP_INFO(node_->get_logger(), "Graph build extra: skipped_no_surface=%zu", skipped_no_surface);
  // RCLCPP_INFO(node_->get_logger(), "Graph build extra: skipped_step_too_high=%zu", skipped_step_too_high);

  std::vector<std::pair<std::string, GraphNode>> node_snapshot;
  node_snapshot.reserve(graph_nodes_.size());
  for (const auto &kv : graph_nodes_) {
    node_snapshot.emplace_back(kv.first, kv.second);
  }

  const size_t min_batch = 128;
  auto pick_thread_count = [&](size_t workload) -> unsigned int {
    if (workload == 0) return 0U;
    if (workload < min_batch) return 1U;
    unsigned int cap = 0U;
    if (worker_thread_limit_ > 0) {
      cap = static_cast<unsigned int>(worker_thread_limit_);
    } else {
      cap = std::thread::hardware_concurrency();
      if (cap == 0U) cap = 2U;
    }
    cap = std::max(1U, cap);
    return std::max(1U, std::min(cap, static_cast<unsigned int>(workload)));
  };

  const std::vector<octomap::point3d> dirs = [](){
    std::vector<octomap::point3d> d;
    for (int dx=-1; dx<=1; ++dx) for (int dy=-1; dy<=1; ++dy) for (int dz=-1; dz<=1; ++dz) {
      if (dx==0 && dy==0 && dz==0) continue;
      d.emplace_back(dx, dy, dz);
    }
    return d;
  }();

  std::vector<std::vector<std::string>> adjacency(node_snapshot.size());

  auto add_edge = [&](size_t from_idx, const std::string &neighbor_id) {
    if (from_idx >= adjacency.size()) return;
    auto &vec = adjacency[from_idx];
    if (std::find(vec.begin(), vec.end(), neighbor_id) == vec.end()) {
      vec.push_back(neighbor_id);
    }
  };

  auto neighbor_worker = [&](size_t start, size_t end) {
    for (size_t idx = start; idx < end; ++idx) {
      const auto &id = node_snapshot[idx].first;
      const GraphNode &node = node_snapshot[idx].second;
      double node_size = node.size;
      // Standard 26-neighborhood sampling (original behavior)
      for (const auto &dir : dirs) {
        octomap::point3d sample = node.center + dir * (node_size * 0.5 + eps);
        if (sample.x() < min_bound_[0] - node_size || sample.x() > max_bound_[0] + node_size ||
            sample.y() < min_bound_[1] - node_size || sample.y() > max_bound_[1] + node_size ||
            sample.z() < min_bound_[2] - node_size || sample.z() > max_bound_[2] + node_size) {
          continue;
        }
  octomap::OcTreeNode* found = octree_->search(sample.x(), sample.y(), sample.z());
  if (!found) continue;
  octomap::OcTreeKey fkey = octree_->coordToKey(sample);
        octomap::point3d found_center = octree_->keyToCoord(fkey);
        double best_dist = std::numeric_limits<double>::infinity();
        size_t best_idx = node_snapshot.size();
        for (size_t j = 0; j < node_snapshot.size(); ++j) {
          const auto &cand = node_snapshot[j].second;
          double dx = cand.center.x() - found_center.x();
          double dy = cand.center.y() - found_center.y();
          double dz = cand.center.z() - found_center.z();
          double dsq = dx*dx + dy*dy + dz*dz;
          if (dsq < best_dist) { best_dist = dsq; best_idx = j; }
        }
        if (best_idx == node_snapshot.size()) continue;
        const auto &best_id = node_snapshot[best_idx].first;
        const auto &best_node = node_snapshot[best_idx].second;
        if (best_node.size + 1e-9 < node_size) continue;
        double thresh = node_size * 1.5;
        if (best_dist > thresh * thresh) {
          ++rejected_distance;
          continue;
        }
        const octomap::point3d &a = node.center;
        const octomap::point3d &b = best_node.center;
        octomap::point3d dir_line = b - a;
        double dist = dir_line.norm();
        bool hit = false;
        if (dist > 1e-9) {
          dir_line /= dist;
          double step_size = std::max(active_voxel_size_, 0.05);
          int steps = std::max(1, static_cast<int>(std::ceil(dist / step_size)));
          for (int si = 1; si < steps; ++si) {
            double t = static_cast<double>(si) / static_cast<double>(steps);
            octomap::point3d s = a + dir_line * (dist * t);
            const double eps_bounds = 1e-6;
            if (s.x() < min_bound_[0] - eps_bounds || s.x() > max_bound_[0] + eps_bounds ||
                s.y() < min_bound_[1] - eps_bounds || s.y() > max_bound_[1] + eps_bounds ||
                s.z() < min_bound_[2] - eps_bounds || s.z() > max_bound_[2] + eps_bounds) continue;
            octomap::OcTreeNode* nn = octree_->search(s.x(), s.y(), s.z());
            if (nn && octree_->isNodeOccupied(nn)) { hit = true; break; }
          }
        }
        if (hit) {
          ++rejected_los;
          continue;
        }
        add_edge(idx, best_id);
      }

      // Optional stair augmentation: search for reachable landings above this node
      if (enable_stair_edges_) {
        const double vertical_window = max_step_height_ + stair_vertical_margin_;
        const double xy_radius = std::max(stair_xy_radius_, active_voxel_size_);
        octomap::point3d min_bb(node.center.x() - xy_radius,
          node.center.y() - xy_radius,
          node.center.z() + 1e-3);
        octomap::point3d max_bb(node.center.x() + xy_radius,
          node.center.y() + xy_radius,
          node.center.z() + vertical_window);
        for (auto it = octree_->begin_leafs_bbx(min_bb, max_bb);
          it != octree_->end_leafs_bbx(); ++it)
        {
          if (!octree_->isNodeOccupied(*it)) continue;
          octomap::point3d occ = it.getCoordinate();
          double landing_z = occ.z() + 0.5 * octree_->getNodeSize(it.getDepth());
          landing_z += std::max(active_voxel_size_, 0.01);
          double dz = landing_z - node.center.z();
          if (dz <= 1e-3 || dz > vertical_window) continue;
          double dx = std::fabs(occ.x() - node.center.x());
          double dy = std::fabs(occ.y() - node.center.y());
          if (dx > xy_radius || dy > xy_radius) continue;

          octomap::point3d landing_center(occ.x(), occ.y(), landing_z);
          double dist = landing_center.distance(node.center);
          if (dist <= 1e-6 || dist > vertical_window + xy_radius) continue;
          octomap::point3d dir_line = landing_center - node.center;
          bool blocked = false;
          double step_size = std::max(active_voxel_size_, 0.05);
          int steps = std::max(1, static_cast<int>(std::ceil(dist / step_size)));
          for (int si = 1; si < steps; ++si)
          {
            double t = static_cast<double>(si) / static_cast<double>(steps);
            octomap::point3d s = node.center + dir_line * t;
            octomap::OcTreeNode* nn = octree_->search(s.x(), s.y(), s.z());
            if (nn && octree_->isNodeOccupied(nn))
            {
              blocked = true;
              break;
            }
          }
          if (blocked) continue;

            octomap::OcTreeKey landing_key = octree_->coordToKey(landing_center);
            auto landing_node_it = std::find_if(node_snapshot.begin(), node_snapshot.end(),
              [&](const auto &pair) {
                if (pair.second.key == landing_key)
                {
                  return true;
                }
                const auto &cand_center = pair.second.center;
                double dx_c = std::fabs(cand_center.x() - landing_center.x());
                double dy_c = std::fabs(cand_center.y() - landing_center.y());
                double dz_c = std::fabs(cand_center.z() - landing_center.z());
                return dx_c <= active_voxel_size_ && dy_c <= active_voxel_size_ && dz_c <= active_voxel_size_;
              });
            if (landing_node_it == node_snapshot.end())
            {
              continue;
            }

            auto landing_idx = static_cast<size_t>(landing_node_it - node_snapshot.begin());
            add_edge(idx, landing_node_it->first);
            add_edge(landing_idx, id);
        }
      }
    }
  };

  if (!node_snapshot.empty()) {
    unsigned int num_threads = pick_thread_count(node_snapshot.size());
    if (num_threads == 0) num_threads = 1;
    if (num_threads == 1) {
      neighbor_worker(0, node_snapshot.size());
    } else {
      size_t chunk = (node_snapshot.size() + num_threads - 1) / num_threads;
      std::vector<std::thread> threads;
      threads.reserve(num_threads);
      for (unsigned int t = 0; t < num_threads; ++t) {
        size_t start = t * chunk;
        if (start >= node_snapshot.size()) break;
        size_t end = std::min(start + chunk, node_snapshot.size());
        threads.emplace_back(neighbor_worker, start, end);
      }
      for (auto &th : threads) {
        if (th.joinable()) th.join();
      }
    }

    graph_adj_.clear();
    for (size_t idx = 0; idx < node_snapshot.size(); ++idx) {
      graph_adj_[node_snapshot[idx].first] = std::move(adjacency[idx]);
    }
  }

  std::unordered_map<std::string, size_t> node_index;
  node_index.reserve(node_snapshot.size());
  for (size_t idx = 0; idx < node_snapshot.size(); ++idx) {
    node_index[node_snapshot[idx].first] = idx;
  }

  std::vector<std::vector<size_t>> neighbor_indices(node_snapshot.size());
  for (size_t idx = 0; idx < node_snapshot.size(); ++idx) {
    const auto &id = node_snapshot[idx].first;
    auto adj_it = graph_adj_.find(id);
    if (adj_it == graph_adj_.end()) continue;
    auto &dest = neighbor_indices[idx];
    dest.reserve(adj_it->second.size());
    for (const auto &nb_id : adj_it->second) {
      auto it_idx = node_index.find(nb_id);
      if (it_idx != node_index.end()) dest.push_back(it_idx->second);
    }
  }

  // After building all adjacency, emit a compact diagnostic summary and check ranges
  if (!graph_nodes_.empty()) {
    double min_cx = std::numeric_limits<double>::infinity();
    double min_cy = std::numeric_limits<double>::infinity();
    double min_cz = std::numeric_limits<double>::infinity();
    double max_cx = -std::numeric_limits<double>::infinity();
    double max_cy = -std::numeric_limits<double>::infinity();
    double max_cz = -std::numeric_limits<double>::infinity();
    size_t out_of_bounds = 0;
    for (const auto &kv : graph_nodes_) {
      const auto &n = kv.second;
  min_cx = std::min(min_cx, static_cast<double>(n.center.x()));
  min_cy = std::min(min_cy, static_cast<double>(n.center.y()));
  min_cz = std::min(min_cz, static_cast<double>(n.center.z()));
  max_cx = std::max(max_cx, static_cast<double>(n.center.x()));
  max_cy = std::max(max_cy, static_cast<double>(n.center.y()));
  max_cz = std::max(max_cz, static_cast<double>(n.center.z()));
      if (n.center.x() < min_bound_[0] - 1e-6 || n.center.x() > max_bound_[0] + 1e-6 ||
          n.center.y() < min_bound_[1] - 1e-6 || n.center.y() > max_bound_[1] + 1e-6 ||
          n.center.z() < min_bound_[2] - 1e-6 || n.center.z() > max_bound_[2] + 1e-6) {
        ++out_of_bounds;
      }
    }
    // RCLCPP_INFO(node_->get_logger(), "Background graph built: nodes=%zu center_x=[%.3f .. %.3f] center_y=[%.3f .. %.3f] center_z=[%.3f .. %.3f] out_of_bounds=%zu",
    //             graph_nodes_.size(), min_cx, max_cx, min_cy, max_cy, min_cz, max_cz, out_of_bounds);
    // RCLCPP_INFO(node_->get_logger(), "Graph adjacency rejects: distance=%zu los=%zu", rejected_distance, rejected_los);
    // Log up to 5 samples
    size_t cnt = 0;
    for (const auto &kv : graph_nodes_) {
      if (cnt++ >= 5) break;
      const auto &n = kv.second;
      // RCLCPP_INFO(node_->get_logger(), "Graph node sample: id=%s center=(%.3f, %.3f, %.3f) size=%.3f",
      //             kv.first.c_str(), n.center.x(), n.center.y(), n.center.z(), n.size);
    }
  }

  // Optionally publish graph node centers as visualization markers for RViz
  if (publish_graph_markers_) {
    publishGraphMarkers();
  }

  auto t_graph_structure_end = std::chrono::steady_clock::now();
  double dur_graph_structure = std::chrono::duration_cast<std::chrono::duration<double>>(t_graph_structure_end - t_graph_start).count();
  RCLCPP_INFO(node_->get_logger(), "Graph build (structure only) took %.3f s (nodes=%zu)", dur_graph_structure, graph_nodes_.size());

  // ---------------------------------------------------------------------------
  // Precompute base node penalties for the whole graph. This avoids recomputing
  // expensive detectors during A* and stabilizes visualization. We compute only
  // base penalties here (no spread/inflation)
  graph_node_penalty_.clear();
  graph_penalized_nodes_.clear();

  if (node_snapshot.empty()) {
    RCLCPP_INFO(node_->get_logger(), "Penalty computation skipped (no nodes)");
  } else {
    auto t_pen_start = std::chrono::steady_clock::now();

  std::vector<double> penalties(node_snapshot.size(), 0.0);
  std::vector<bool> penalized(node_snapshot.size(), false);
  std::vector<bool> spread_sources(node_snapshot.size(), false);

    auto compute_penalty = [&](size_t idx) {
  const auto &nid = node_snapshot[idx].first;
  const auto &n = node_snapshot[idx].second;
  double penalty = 0.0;
  bool corner_like = false;

      // centroid-shift detector (k-nearest variant)
      std::vector<octomap::point3d> nbrs;
      nbrs.reserve(32);
      for (size_t j = 0; j < node_snapshot.size(); ++j) {
        if (j == idx) continue;
        const auto &nj = node_snapshot[j].second;
        double dx = nj.center.x() - n.center.x();
        double dy = nj.center.y() - n.center.y();
        double dxy = std::sqrt(dx*dx + dy*dy);
        if (dxy <= ransac_radius_) nbrs.push_back(nj.center);
      }
      if (nbrs.size() >= 4) {
        std::sort(nbrs.begin(), nbrs.end(), [&](const octomap::point3d &a, const octomap::point3d &b){
          double da = std::hypot(a.x()-n.center.x(), a.y()-n.center.y());
          double db = std::hypot(b.x()-n.center.x(), b.y()-n.center.y());
          return da < db;
        });
        size_t k_use = std::min(static_cast<size_t>(centroid_k_), nbrs.size());
        double cx=0, cy=0, cz=0;
        for (size_t ii=0; ii<k_use; ++ii) { cx += nbrs[ii].x(); cy += nbrs[ii].y(); cz += nbrs[ii].z(); }
        cx /= static_cast<double>(k_use); cy /= static_cast<double>(k_use); cz /= static_cast<double>(k_use);
        double shift = std::sqrt((cx - n.center.x())*(cx - n.center.x()) + (cy - n.center.y())*(cy - n.center.y()) + (cz - n.center.z())*(cz - n.center.z()));
        double Zi = 1e9; for (size_t ii=0; ii<k_use; ++ii) {
          double dd = std::sqrt((nbrs[ii].x()-n.center.x())*(nbrs[ii].x()-n.center.x()) + (nbrs[ii].y()-n.center.y())*(nbrs[ii].y()-n.center.y()) + (nbrs[ii].z()-n.center.z())*(nbrs[ii].z()-n.center.z()));
          Zi = std::min(Zi, dd);
        }
        if (Zi < 1e-6) Zi = n.size;
        if (shift > centroid_lambda_ * Zi) {
          penalty += std::max(0.0, corner_penalty_weight_);
          corner_like = true;
        } else {
          double minx=1e9,maxx=-1e9,miny=1e9,maxy=-1e9,minz=1e9,maxz=-1e9;
          for (size_t ii=0; ii<k_use; ++ii) {
            double vx = nbrs[ii].x(); double vy = nbrs[ii].y(); double vz = nbrs[ii].z();
            if (vx < minx) minx = vx; if (vx > maxx) maxx = vx;
            if (vy < miny) miny = vy; if (vy > maxy) maxy = vy;
            if (vz < minz) minz = vz; if (vz > maxz) maxz = vz;
          }
          int axes = 0; double rho = 0.05;
          if ((maxx - minx) > rho) ++axes; if ((maxy - miny) > rho) ++axes; if ((maxz - minz) > rho) ++axes;
          if (axes >= 2) { penalty += 0.5 * centroid_penalty_weight_; }
        }
      }

      penalties[idx] = penalty;
      penalized[idx] = penalty > 1e-9;
      if (corner_like) spread_sources[idx] = true;
    };

    auto worker = [&](size_t start, size_t end) {
      for (size_t idx = start; idx < end; ++idx) {
        compute_penalty(idx);
      }
    };

    unsigned int num_threads = pick_thread_count(node_snapshot.size());
    if (num_threads == 0) num_threads = 1;

    if (num_threads == 1) {
      worker(0, node_snapshot.size());
    } else {
      size_t chunk = (node_snapshot.size() + num_threads - 1) / num_threads;
      std::vector<std::thread> threads;
      threads.reserve(num_threads);
      for (unsigned int t = 0; t < num_threads; ++t) {
        size_t start = t * chunk;
        if (start >= node_snapshot.size()) break;
        size_t end = std::min(start + chunk, node_snapshot.size());
        threads.emplace_back(worker, start, end);
      }
      for (auto &th : threads) {
        if (th.joinable()) th.join();
      }
    }
    //  Apply penalty spread from corner-like nodes to their neighbors
    if (penalty_spread_radius_ > 1e-6 && penalty_spread_factor_ > 1e-6) {
      const double max_radius = penalty_spread_radius_;
      for (size_t src = 0; src < node_snapshot.size(); ++src) {
        double base_pen = penalties[src];
        if (base_pen <= 1e-9) continue;
        if (!spread_sources[src]) continue;
        for (size_t nb_idx : neighbor_indices[src]) {
          const auto &a = node_snapshot[src].second.center;
          const auto &b = node_snapshot[nb_idx].second.center;
          double dist = a.distance(b);
          if (!std::isfinite(dist)) continue;
          if (dist > max_radius) continue;
          double atten = 1.0 - (dist / max_radius);
          if (atten < 0.0) atten = 0.0;
          double addition = base_pen * penalty_spread_factor_ * atten;
          if (addition <= 0.0) continue;
          penalties[nb_idx] += addition;
          penalized[nb_idx] = true;
        }
      }
    }

    for (size_t idx = 0; idx < node_snapshot.size(); ++idx) {
      const auto &nid = node_snapshot[idx].first;
      double penalty = penalties[idx];
      graph_node_penalty_[nid] = penalty;
      if (penalized[idx]) {
        graph_penalized_nodes_.insert(nid);
      }
    }

    auto t_pen_end = std::chrono::steady_clock::now();
    double dur_pen = std::chrono::duration_cast<std::chrono::duration<double>>(t_pen_end - t_pen_start).count();
    RCLCPP_INFO(node_->get_logger(), "Penalty computation took %.3f s using %u thread(s)", dur_pen, std::max(1U, num_threads));
  }

  auto t_graph_end = std::chrono::steady_clock::now();
  double dur_graph_total = std::chrono::duration_cast<std::chrono::duration<double>>(t_graph_end - t_graph_start).count();
  RCLCPP_INFO(node_->get_logger(), "Total buildConnectivityGraph() took %.3f s", dur_graph_total);

  // After a full build, mark all occupied voxels as processed
  processed_occupied_keys_.clear();
  for (auto it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it) {
    if (octree_->isNodeOccupied(*it)) {
      auto c = it.getCoordinate();
      char key_buf[128];
      int ix_mm = static_cast<int>(std::round(c.x() * 1000.0));
      int iy_mm = static_cast<int>(std::round(c.y() * 1000.0));
      int iz_mm = static_cast<int>(std::round(c.z() * 1000.0));
      std::snprintf(key_buf, sizeof(key_buf), "occ_%d_%d_%d", ix_mm, iy_mm, iz_mm);
      processed_occupied_keys_.insert(std::string(key_buf));
    }
  }
  nodes_needing_adjacency_update_.clear();
}

void AstarOctoPlanner::updateConnectivityGraphIncremental(double eps)
{
  auto t_start = std::chrono::steady_clock::now();
  if (!octree_) return;

  // Collect new occupied voxels that we haven't processed yet
  std::vector<std::tuple<octomap::point3d, octomap::OcTreeKey, unsigned int, double>> new_voxels;

  for (auto it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it) {
    if (!octree_->isNodeOccupied(*it)) continue;

    octomap::point3d occ = it.getCoordinate();
    char key_buf[128];
    int ix_mm = static_cast<int>(std::round(occ.x() * 1000.0));
    int iy_mm = static_cast<int>(std::round(occ.y() * 1000.0));
    int iz_mm = static_cast<int>(std::round(occ.z() * 1000.0));
    std::snprintf(key_buf, sizeof(key_buf), "occ_%d_%d_%d", ix_mm, iy_mm, iz_mm);
    std::string voxel_key(key_buf);

    if (processed_occupied_keys_.find(voxel_key) == processed_occupied_keys_.end()) {
      // This is a new occupied voxel
      unsigned int depth = it.getDepth();
      double node_size = octree_->getNodeSize(depth);
      new_voxels.emplace_back(occ, octree_->coordToKey(occ), depth, node_size);
      processed_occupied_keys_.insert(voxel_key);
    }
  }

  if (new_voxels.empty()) {
    RCLCPP_DEBUG(node_->get_logger(), "Incremental update: no new voxels to process");
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Incremental update: processing %zu new voxels", new_voxels.size());

  // Create graph nodes for each new occupied voxel (surface nodes above occupied cells)
  std::vector<std::pair<std::string, GraphNode>> new_nodes;

  for (const auto& [occ, key, depth, node_size] : new_voxels) {
    double node_z = occ.z() + 0.5 * node_size;
    node_z += std::max(active_voxel_size_, 0.01);

    if (!std::isfinite(occ.x()) || !std::isfinite(occ.y()) || !std::isfinite(node_z)) continue;
    if (occ.x() < min_bound_[0] - 1e-6 || occ.x() > max_bound_[0] + 1e-6 ||
        occ.y() < min_bound_[1] - 1e-6 || occ.y() > max_bound_[1] + 1e-6 ||
        node_z < min_bound_[2] - 1e-6 || node_z > max_bound_[2] + 1e-6) continue;

    // Vertical clearance check above surface
    bool vertical_clear = true;
    double stepz = std::max(active_voxel_size_, 0.05);
    for (double z = node_z; z <= node_z + robot_height_; z += stepz) {
      octomap::OcTreeNode* nn = octree_->search(occ.x(), occ.y(), z);
      if (nn && octree_->isNodeOccupied(nn)) { vertical_clear = false; break; }
    }
    if (!vertical_clear) continue;

    octomap::point3d center(occ.x(), occ.y(), node_z);
    octomap::OcTreeKey center_key = octree_->coordToKey(center);

    GraphNode gn;
    gn.key = center_key;
    gn.depth = depth;
    gn.center = center;
    gn.size = node_size;

    // Create stable id
    char idbuf[128];
    int node_ix_mm = static_cast<int>(std::round(center.x() * 1000.0));
    int node_iy_mm = static_cast<int>(std::round(center.y() * 1000.0));
    int node_iz_mm = static_cast<int>(std::round(center.z() * 1000.0));
    std::snprintf(idbuf, sizeof(idbuf), "c_%d_%d_%d", node_ix_mm, node_iy_mm, node_iz_mm);
    std::string node_id(idbuf);

    // Only add if not already in the graph
    if (graph_nodes_.find(node_id) == graph_nodes_.end()) {
      graph_nodes_.emplace(node_id, gn);
      graph_adj_[node_id] = std::vector<std::string>(); // Initialize empty adjacency
      new_nodes.emplace_back(node_id, gn);
      nodes_needing_adjacency_update_.insert(node_id);
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Incremental update: added %zu new graph nodes (total nodes: %zu)",
              new_nodes.size(), graph_nodes_.size());

  if (new_nodes.empty() && nodes_needing_adjacency_update_.empty()) {
    return; // Nothing to update
  }

  // Build adjacency for new nodes and update existing nodes that may connect to new ones
  const std::vector<octomap::point3d> dirs = [](){
    std::vector<octomap::point3d> d;
    for (int dx=-1; dx<=1; ++dx) for (int dy=-1; dy<=1; ++dy) for (int dz=-1; dz<=1; ++dz) {
      if (dx==0 && dy==0 && dz==0) continue;
      d.emplace_back(dx, dy, dz);
    }
    return d;
  }();

  // Create a snapshot of nodes that need adjacency updates
  std::vector<std::pair<std::string, GraphNode>> nodes_to_update;
  for (const auto& node_id : nodes_needing_adjacency_update_) {
    auto it = graph_nodes_.find(node_id);
    if (it != graph_nodes_.end()) {
      nodes_to_update.emplace_back(it->first, it->second);
    }
  }

  // Also include existing nodes that are close to new nodes (they may need new edges)
  double neighbor_search_radius = active_voxel_size_ * 3.0; // Search nearby existing nodes
  for (const auto& [new_id, new_node] : new_nodes) {
    for (const auto& [existing_id, existing_node] : graph_nodes_) {
      if (existing_id == new_id) continue;
      if (nodes_needing_adjacency_update_.find(existing_id) != nodes_needing_adjacency_update_.end()) continue;

      double dist = new_node.center.distance(existing_node.center);
      if (dist <= neighbor_search_radius) {
        nodes_needing_adjacency_update_.insert(existing_id);
        nodes_to_update.emplace_back(existing_id, existing_node);
      }
    }
  }

  // Build node index for fast lookups
  std::unordered_map<std::string, size_t> node_index;
  std::vector<std::pair<std::string, GraphNode>> all_nodes_snapshot;
  all_nodes_snapshot.reserve(graph_nodes_.size());
  for (const auto& kv : graph_nodes_) {
    node_index[kv.first] = all_nodes_snapshot.size();
    all_nodes_snapshot.emplace_back(kv.first, kv.second);
  }

  // Update adjacency for affected nodes
  for (const auto& [node_id, node] : nodes_to_update) {
    std::vector<std::string>& adj = graph_adj_[node_id];

    for (const auto& dir : dirs) {
      octomap::point3d sample = node.center + dir * (node.size * 0.5 + eps);

      if (sample.x() < min_bound_[0] - node.size || sample.x() > max_bound_[0] + node.size ||
          sample.y() < min_bound_[1] - node.size || sample.y() > max_bound_[1] + node.size ||
          sample.z() < min_bound_[2] - node.size || sample.z() > max_bound_[2] + node.size) {
        continue;
      }

      octomap::OcTreeNode* found = octree_->search(sample.x(), sample.y(), sample.z());
      if (!found) continue;

      octomap::OcTreeKey fkey = octree_->coordToKey(sample);
      octomap::point3d found_center = octree_->keyToCoord(fkey);

      // Find the closest graph node to this sample point
      double best_dist = std::numeric_limits<double>::infinity();
      std::string best_id;
      for (const auto& [cand_id, cand_node] : all_nodes_snapshot) {
        double dx = cand_node.center.x() - found_center.x();
        double dy = cand_node.center.y() - found_center.y();
        double dz = cand_node.center.z() - found_center.z();
        double dsq = dx*dx + dy*dy + dz*dz;
        if (dsq < best_dist) { best_dist = dsq; best_id = cand_id; }
      }

      if (best_id.empty() || best_id == node_id) continue;

      double thresh = node.size * 1.5;
      if (best_dist > thresh * thresh) continue;

      // Line-of-sight check
      const auto& best_node = graph_nodes_.at(best_id);
      const octomap::point3d& a = node.center;
      const octomap::point3d& b = best_node.center;
      octomap::point3d dir_line = b - a;
      double dist = dir_line.norm();
      bool hit = false;

      if (dist > 1e-9) {
        dir_line /= dist;
        double step_size = std::max(active_voxel_size_, 0.05);
        int steps = std::max(1, static_cast<int>(std::ceil(dist / step_size)));
        for (int si = 1; si < steps; ++si) {
          double t = static_cast<double>(si) / static_cast<double>(steps);
          octomap::point3d s = a + dir_line * (dist * t);
          octomap::OcTreeNode* nn = octree_->search(s.x(), s.y(), s.z());
          if (nn && octree_->isNodeOccupied(nn)) { hit = true; break; }
        }
      }

      if (hit) continue;

      // Add edge if not already present
      if (std::find(adj.begin(), adj.end(), best_id) == adj.end()) {
        adj.push_back(best_id);
      }
      // Add reverse edge
      auto& reverse_adj = graph_adj_[best_id];
      if (std::find(reverse_adj.begin(), reverse_adj.end(), node_id) == reverse_adj.end()) {
        reverse_adj.push_back(node_id);
      }
    }
  }

  // Clear the pending update set
  nodes_needing_adjacency_update_.clear();

  // Update penalties for new nodes (simplified - just using centroid shift for new nodes)
  for (const auto& [node_id, node] : new_nodes) {
    // Start with zero penalty, can be computed more thoroughly if needed
    graph_node_penalty_[node_id] = 0.0;
  }

  // Publish updated markers
  if (publish_graph_markers_) {
    publishGraphMarkers();
  }

  auto t_end = std::chrono::steady_clock::now();
  double dur = std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start).count();
  RCLCPP_INFO(node_->get_logger(), "Incremental graph update took %.3f s", dur);
}

void AstarOctoPlanner::publishGraphMarkers()
{
  if (!graph_marker_pub_ || graph_nodes_.empty()) return;
  visualization_msgs::msg::MarkerArray ma;
  visualization_msgs::msg::Marker m;
  m.header.frame_id = map_frame_.empty() ? "map" : map_frame_;
  m.header.stamp = node_->now();
  m.ns = "graph_nodes";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::CUBE_LIST;
  m.action = visualization_msgs::msg::Marker::ADD;
  // set scale for cubes (use node size or default to active voxel size if available)
  double scale = active_voxel_size_ > 0.0 ? std::max(active_voxel_size_, 0.05) : 0.05;
  m.scale.x = static_cast<float>(scale);
  m.scale.y = static_cast<float>(scale);
  m.scale.z = static_cast<float>(scale);
  // color
  m.color.r = 0.0f;
  m.color.g = 1.0f;
  m.color.b = 0.0f;
  m.color.a = 0.8f;

  for (const auto &kv : graph_nodes_) {
    geometry_msgs::msg::Point p;
    p.x = kv.second.center.x();
    p.y = kv.second.center.y();
    p.z = kv.second.center.z();
    m.points.push_back(p);
  }
  ma.markers.push_back(m);
  graph_marker_pub_->publish(ma);
  //RCLCPP_INFO(node_->get_logger(), "Published graph markers: markers=%zu points=%zu scale=%.3f", ma.markers.size(), ma.markers.empty() ? 0 : ma.markers[0].points.size(), scale);
}

std::string AstarOctoPlanner::findClosestGraphNode(const octomap::point3d& p) const
{
  if (graph_nodes_.empty()) return std::string();
  double best = std::numeric_limits<double>::infinity();
  std::string best_id;
  for (const auto &kv : graph_nodes_) {
    double dx = kv.second.center.x() - p.x();
    double dy = kv.second.center.y() - p.y();
    double dz = kv.second.center.z() - p.z();
    double d = dx*dx + dy*dy + dz*dz;
    if (d < best) { best = d; best_id = kv.first; }
  }
  return best_id;
}

std::vector<std::string> AstarOctoPlanner::planOnGraph(const std::string& start_id, const std::string& goal_id) const
{
  std::vector<std::string> empty;
  if (start_id.empty() || goal_id.empty()) return empty;
  if (graph_nodes_.find(start_id) == graph_nodes_.end() || graph_nodes_.find(goal_id) == graph_nodes_.end()) return empty;

  struct Item { std::string id; double f; double g; };
  struct Cmp { bool operator()(const Item&a, const Item&b) const { return a.f > b.f; } };

  std::priority_queue<Item, std::vector<Item>, Cmp> open;
  std::unordered_map<std::string, double> gscore;
  std::unordered_map<std::string, std::string> came_from;

  auto heuristic = [this,&goal_id](const std::string &a)->double{
    if (graph_nodes_.find(a) == graph_nodes_.end() || graph_nodes_.find(goal_id) == graph_nodes_.end()) return 0.0;
    const auto &pa = graph_nodes_.at(a).center;
    const auto &pg = graph_nodes_.at(goal_id).center;
    double dx = pa.x()-pg.x(); double dy = pa.y()-pg.y(); double dz = pa.z()-pg.z();
    return std::sqrt(dx*dx+dy*dy+dz*dz);
  };

  open.push({start_id, heuristic(start_id), 0.0});
  gscore[start_id] = 0.0;

  while (!open.empty()) {
    Item cur = open.top(); open.pop();
    if (cur.id == goal_id) {
      // reconstruct
      std::vector<std::string> path;
      std::string u = goal_id;
      while (came_from.find(u) != came_from.end()) { path.push_back(u); u = came_from.at(u); }
      path.push_back(start_id);
      std::reverse(path.begin(), path.end());
      return path;
    }
    if (gscore.find(cur.id) != gscore.end() && cur.g > gscore.at(cur.id)) continue; // stale
    auto it = graph_adj_.find(cur.id);
    if (it == graph_adj_.end()) continue;
    for (const std::string &nb : it->second) {
      if (graph_nodes_.find(cur.id) == graph_nodes_.end() || graph_nodes_.find(nb) == graph_nodes_.end()) continue;
      double tentative = cur.g + (graph_nodes_.at(cur.id).center.distance(graph_nodes_.at(nb).center));
      if (gscore.find(nb) == gscore.end() || tentative < gscore[nb]) {
        came_from[nb] = cur.id;
        gscore[nb] = tentative;
        open.push({nb, tentative + heuristic(nb), tentative});
      }
    }
  }

  return empty;
}

// Legacy grid-based astar() removed — graph-based planning is used (planOnGraph).

// Convert a grid index triple into world coordinates (meters)
std::array<double, 3> AstarOctoPlanner::gridToWorld(const std::tuple<int, int, int>& grid_pt)
{
  int ix, iy, iz;
  std::tie(ix, iy, iz) = grid_pt;
  double vx = active_voxel_size_ > 0.0 ? active_voxel_size_ : voxel_size_;
  double wx = min_bound_[0] + static_cast<double>(ix) * vx;
  double wy = min_bound_[1] + static_cast<double>(iy) * vx;
  double wz = min_bound_[2] + static_cast<double>(iz) * vx;
  return {wx, wy, wz};
}

// The following helpers were removed from the source as they are no longer
// referenced by the graph-based planner implementation: worldToGrid,
// isWithinBounds, and isOccupied. Their declarations were removed from the
// public header to avoid exposing unused private helpers.

// Very simple cylinder collision check: sample a few radial directions at the given radius
// and ensure no occupied node is found at those sample points (at the same z).
bool AstarOctoPlanner::isCylinderCollisionFree(const std::tuple<int, int, int>& coord, double radius)
{
  // Conservative rectangular footprint check: sample points within the robot footprint
  // across several height slices and return false if any sample collides with an occupied node.
  if (!octree_) return true;
  auto w = gridToWorld(coord);
  double half_w = 0.5 * (robot_width_ + footprint_margin_);
  double half_l = 0.5 * (robot_length_ + footprint_margin_);
  // number of samples along length and width (runtime-configurable)
  const int nx = std::max(1, footprint_samples_x_);
  const int ny = std::max(1, footprint_samples_y_);
  // height sampling across robot height
  double z_bottom = w[2];
  double z_top = w[2] + robot_height_;
  const int nz = std::max(1, static_cast<int>(std::ceil(robot_height_ / std::max(0.05, active_voxel_size_))));
  for (int iz = 0; iz < nz; ++iz) {
    double frac = nz == 1 ? 0.5 : static_cast<double>(iz) / static_cast<double>(nz - 1);
    double z = z_bottom + frac * (z_top - z_bottom);
    for (int ix = 0; ix < nx; ++ix) {
      double fx = nx == 1 ? 0.0 : (static_cast<double>(ix) / static_cast<double>(nx - 1) * 2.0 - 1.0);
      double sx = w[0] + fx * half_l;
      for (int iy = 0; iy < ny; ++iy) {
        double fy = ny == 1 ? 0.0 : (static_cast<double>(iy) / static_cast<double>(ny - 1) * 2.0 - 1.0);
        double sy = w[1] + fy * half_w;
        octomap::OcTreeNode* n = octree_->search(sx, sy, z);
        if (n && octree_->isNodeOccupied(n)) return false;
      }
    }
  }
  // also check center column
  for (double z = z_bottom; z <= z_top; z += std::max(active_voxel_size_, 0.05)) {
    octomap::OcTreeNode* c = octree_->search(w[0], w[1], z);
    if (c && octree_->isNodeOccupied(c)) return false;
  }
  return true;
}

void AstarOctoPlanner::clearOccupiedCylinderAround(const geometry_msgs::msg::Point &center, double radius, double z_bottom, double z_top)
{
  if (!octree_) return;
  // sample a grid over the cylinder footprint using current active voxel size
  double step = std::max(active_voxel_size_, 0.05);
  int nx = static_cast<int>(std::ceil((2.0 * radius) / step));
  int ny = nx;
  for (int ix = -nx; ix <= nx; ++ix) {
    for (int iy = -ny; iy <= ny; ++iy) {
      double x = center.x + ix * step;
      double y = center.y + iy * step;
      double dx = x - center.x;
      double dy = y - center.y;
      if ((dx*dx + dy*dy) > (radius+step)*(radius+step)) continue;
      for (double z = z_bottom; z <= z_top; z += step) {
        octomap::OcTreeNode* n = octree_->search(x, y, z);
        if (n && octree_->isNodeOccupied(n)) {
          // set node as free by updating occupancy to 0.0 probability
          octree_->updateNode(octomap::point3d(x,y,z), false);
        }
      }
    }
  }
}

// hasNoOccupiedCellsAbove: implementation removed (legacy helper not used)

// pointcloud2Callback removed: planner now uses octree messages exclusively.

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
  // Subscribe to octomap binary topic (primary map source)

  // Declare and use a configurable octomap topic name (allows changing topic at runtime)
  octomap_topic_ = node_->declare_parameter(name_ + ".octomap_topic", octomap_topic_);
  octomap_sub_ = node_->create_subscription<octomap_msgs::msg::Octomap>(
    octomap_topic_, 1,
    std::bind(&AstarOctoPlanner::octomapCallback, this, std::placeholders::_1));

  // parameter: enable/disable processing of incoming octomap updates while
  // keeping the subscription alive (useful to lock the map after mapping pass)
  enable_octomap_updates_ = node_->declare_parameter(name_ + ".enable_octomap_updates", enable_octomap_updates_);

  // parameter: enable/disable incremental graph building (merge new octomap data
  // into existing octree and incrementally update the graph instead of rebuilding)
  incremental_graph_build_ = node_->declare_parameter(name_ + ".incremental_graph_build", incremental_graph_build_);

  // Declare planner tuning parameters (can be changed at runtime)
  z_threshold_ = node_->declare_parameter(name_ + ".z_threshold", z_threshold_);
  robot_radius_ = node_->declare_parameter(name_ + ".robot_radius", robot_radius_);
  min_vertical_clearance_ = node_->declare_parameter(name_ + ".min_vertical_clearance", min_vertical_clearance_);
  max_vertical_clearance_ = node_->declare_parameter(name_ + ".max_vertical_clearance", max_vertical_clearance_);
  // robot footprint params (runtime-tunable)
  robot_width_ = node_->declare_parameter(name_ + ".robot_width", robot_width_);
  robot_length_ = node_->declare_parameter(name_ + ".robot_length", robot_length_);
  robot_height_ = node_->declare_parameter(name_ + ".robot_height", robot_height_);
  footprint_margin_ = node_->declare_parameter(name_ + ".footprint_margin", footprint_margin_);
  footprint_samples_x_ = node_->declare_parameter(name_ + ".footprint_samples_x", footprint_samples_x_);
  footprint_samples_y_ = node_->declare_parameter(name_ + ".footprint_samples_y", footprint_samples_y_);
  max_surface_distance_ = node_->declare_parameter(name_ + ".max_surface_distance", max_surface_distance_);
  max_step_height_ = node_->declare_parameter(name_ + ".max_step_height", max_step_height_);
  enable_stair_edges_ = node_->declare_parameter(name_ + ".enable_stair_edges", enable_stair_edges_);
  stair_xy_radius_ = node_->declare_parameter(name_ + ".stair_xy_radius", stair_xy_radius_);
  stair_vertical_margin_ = node_->declare_parameter(name_ + ".stair_vertical_margin", stair_vertical_margin_);
  // corner/edge penalty parameters
  corner_radius_ = node_->declare_parameter(name_ + ".corner_radius", 0.40);
  corner_penalty_weight_ = node_->declare_parameter(name_ + ".corner_penalty_weight", corner_penalty_weight_);
  // Sector-histogram detector parameters (for directional wall/corner detection)
  sector_bins_ = node_->declare_parameter(name_ + ".sector_bins", sector_bins_);
  sector_radius_ = node_->declare_parameter(name_ + ".sector_radius", sector_radius_);
  sector_peak_thresh_ = node_->declare_parameter(name_ + ".sector_peak_thresh", sector_peak_thresh_);
  sector_min_inliers_ = node_->declare_parameter(name_ + ".sector_min_inliers", sector_min_inliers_);
  centroid_k_ = node_->declare_parameter(name_ + ".centroid_k", centroid_k_);
  centroid_lambda_ = node_->declare_parameter(name_ + ".centroid_lambda", centroid_lambda_);
  centroid_penalty_weight_ = node_->declare_parameter(name_ + ".centroid_penalty_weight", centroid_penalty_weight_);

  // Penalty spread parameters (spread detected penalties to neighbors)
  penalty_spread_radius_ = node_->declare_parameter(name_ + ".penalty_spread_radius", penalty_spread_radius_);
  penalty_spread_factor_ = node_->declare_parameter(name_ + ".penalty_spread_factor", penalty_spread_factor_);
  worker_thread_limit_ = node_->declare_parameter(name_ + ".worker_thread_limit", worker_thread_limit_);


  // Graph marker publishing (for RViz visualization of node centers)
    publish_graph_markers_ = node_->declare_parameter(name_ + ".publish_graph_markers", publish_graph_markers_);
  // always create publisher so we can toggle publishing at runtime without creating publishers later
  // use transient_local so RViz or late subscribers receive the last published markers
  graph_marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("~/graph_nodes", rclcpp::QoS(1).transient_local());

  RCLCPP_INFO(node_->get_logger(), "Declared parameters: %s.z_threshold=%.3f, %s.robot_radius=%.3f, %s.min_vertical_clearance=%.3f, %s.max_vertical_clearance=%.3f",
               name_.c_str(), z_threshold_, name_.c_str(), robot_radius_, name_.c_str(), min_vertical_clearance_, name_.c_str(), max_vertical_clearance_);

  reconfiguration_callback_handle_ = node_->add_on_set_parameters_callback(
      std::bind(&AstarOctoPlanner::reconfigureCallback, this, std::placeholders::_1));

  // Create a periodic timer to (re)build the connectivity graph in background
  graph_build_timer_ = node_->create_wall_timer(
    std::chrono::seconds(1),
    [this]() {
      if (!enable_octomap_updates_) return; // do nothing when updates disabled
      if (!graph_dirty_) return;
      std::lock_guard<std::mutex> lock(graph_mutex_);
      if (!graph_dirty_) return;

      // Use incremental update if we have an existing graph and incremental mode is enabled
      if (incremental_graph_build_ && !graph_nodes_.empty()) {
        RCLCPP_INFO(node_->get_logger(), "Background: incrementally updating connectivity graph...");
        updateConnectivityGraphIncremental();
        graph_dirty_ = false;
        RCLCPP_INFO(node_->get_logger(), "Background: connectivity graph updated. Nodes=%zu", graph_nodes_.size());
      } else {
        RCLCPP_INFO(node_->get_logger(), "Background: rebuilding connectivity graph from scratch...");
        buildConnectivityGraph();
        graph_dirty_ = false;
        RCLCPP_INFO(node_->get_logger(), "Background: connectivity graph ready. Nodes=%zu", graph_nodes_.size());
      }
    }
  );

  return true;
}

// Helper: traverse the given octree and return the node (may be internal)
// that contains the provided world coordinate. If a child for the
// coordinate doesn't exist at some level, return the current internal node
// (coarse cell).
// octomap::OcTreeNode* AstarOctoPlanner::findNodeByCoordinateInTree(const octomap::point3d& coord)
// {
//   if (!octree_) return nullptr;
//
//   octomap::OcTreeNode* current = octree_->getRoot();
//   if (!current) return nullptr;
//
//   unsigned int maxDepth = octree_->getTreeDepth();
//
//     // Use octree's search convenience method. It will return the finest node
//     // available for the coordinate (may be internal if the tree is pruned).
//     octomap::OcTreeNode* node = octree_->search(coord.x(), coord.y(), coord.z());
//     if (node) return node;
//
//     // Fallback: return the root node if search failed (coarse cell).
//     return octree_->getRoot();
// }

void AstarOctoPlanner::octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
  // Deserialize octomap message into an AbstractOcTree
  octomap::AbstractOcTree* tree_ptr = octomap_msgs::fullMsgToMap(*msg);
  if (!tree_ptr) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to convert Octomap message to AbstractOcTree");
    return;
  }

  // Respect runtime toggle: if updates are disabled, drop incoming maps
  if (!enable_octomap_updates_) {
    //RCLCPP_INFO(node_->get_logger(), "enable_octomap_updates is false — ignoring incoming octomap message.");
    delete tree_ptr;
    return;
  }

  octomap::OcTree* incoming_tree = dynamic_cast<octomap::OcTree*>(tree_ptr);
  if (!incoming_tree) {
    RCLCPP_ERROR(node_->get_logger(), "Octomap message is not an OcTree type");
    delete tree_ptr;
    return;
  }

  // Apply sensor model / occupancy parameters to incoming tree
  incoming_tree->setProbHit(octomap_prob_hit_);
  incoming_tree->setProbMiss(octomap_prob_miss_);
  incoming_tree->setOccupancyThres(octomap_thres_);
  incoming_tree->setClampingThresMin(octomap_clamp_min_);
  incoming_tree->setClampingThresMax(octomap_clamp_max_);

  // Store the frame id of the incoming map
  map_frame_ = msg->header.frame_id;

  // Incremental merge: if we already have an octree and incremental mode is enabled,
  // merge the incoming data into our existing octree instead of replacing it
  if (incremental_graph_build_ && octree_) {
    // Merge incoming octree into existing one
    // Iterate through all leaves in the incoming tree and insert them into our octree
    size_t merged_count = 0;
    for (auto it = incoming_tree->begin_leafs(), end = incoming_tree->end_leafs(); it != end; ++it) {
      octomap::point3d coord = it.getCoordinate();
      float log_odds = it->getLogOdds();

      // Update the node in our existing octree with the log-odds value
      octomap::OcTreeNode* node = octree_->updateNode(coord, log_odds > 0);
      if (node) {
        node->setLogOdds(log_odds);
        ++merged_count;
      }
    }
    delete incoming_tree;
    RCLCPP_DEBUG(node_->get_logger(), "Merged %zu voxels into existing octree", merged_count);
  } else {
    // First time or non-incremental mode: replace the octree entirely
    octree_.reset(incoming_tree);
    // Clear tracking sets since we're starting fresh
    processed_occupied_keys_.clear();
    nodes_needing_adjacency_update_.clear();
  }

  // Update bounds incrementally: only expand, never shrink
  active_voxel_size_ = octree_->getResolution();
  double expand = std::max(1.0, 5.0 * active_voxel_size_); // at least 1 meter margin

  // Start with existing bounds (or initialize if first time)
  double min_x = min_bound_[0];
  double min_y = min_bound_[1];
  double min_z = min_bound_[2];
  double max_x = max_bound_[0];
  double max_y = max_bound_[1];
  double max_z = max_bound_[2];

  // Check if this is the first initialization (bounds still at defaults)
  bool first_init = (min_x == 0.0 && min_y == 0.0 && min_z == 0.0 &&
                     max_x == 0.0 && max_y == 0.0 && max_z == 0.0);
  if (first_init) {
    min_x = std::numeric_limits<double>::infinity();
    min_y = std::numeric_limits<double>::infinity();
    min_z = std::numeric_limits<double>::infinity();
    max_x = -std::numeric_limits<double>::infinity();
    max_y = -std::numeric_limits<double>::infinity();
    max_z = -std::numeric_limits<double>::infinity();
  }

  size_t occupied = 0;
  size_t new_voxels = 0;
  for (auto it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it) {
    if (octree_->isNodeOccupied(*it)) {
      auto c = it.getCoordinate();
      double cx = static_cast<double>(c.x());
      double cy = static_cast<double>(c.y());
      double cz = static_cast<double>(c.z());

      // Track new occupied voxels for incremental graph updates
      char key_buf[128];
      int ix_mm = static_cast<int>(std::round(cx * 1000.0));
      int iy_mm = static_cast<int>(std::round(cy * 1000.0));
      int iz_mm = static_cast<int>(std::round(cz * 1000.0));
      std::snprintf(key_buf, sizeof(key_buf), "occ_%d_%d_%d", ix_mm, iy_mm, iz_mm);
      std::string voxel_key(key_buf);

      if (processed_occupied_keys_.find(voxel_key) == processed_occupied_keys_.end()) {
        // This is a new voxel we haven't processed before
        ++new_voxels;
        // Mark the corresponding graph node (to be created) as needing adjacency update
        double node_z = cz + 0.5 * octree_->getNodeSize(it.getDepth()) + std::max(active_voxel_size_, 0.01);
        int node_ix_mm = static_cast<int>(std::round(cx * 1000.0));
        int node_iy_mm = static_cast<int>(std::round(cy * 1000.0));
        int node_iz_mm = static_cast<int>(std::round(node_z * 1000.0));
        char node_key_buf[128];
        std::snprintf(node_key_buf, sizeof(node_key_buf), "c_%d_%d_%d", node_ix_mm, node_iy_mm, node_iz_mm);
        nodes_needing_adjacency_update_.insert(std::string(node_key_buf));
      }

      // Expand bounds if needed
      min_x = std::min(min_x, cx - expand);
      min_y = std::min(min_y, cy - expand);
      min_z = std::min(min_z, cz - expand);
      max_x = std::max(max_x, cx + expand);
      max_y = std::max(max_y, cy + expand);
      max_z = std::max(max_z, cz + expand);
      ++occupied;
    }
  }

  if (occupied == 0 && first_init) {
    RCLCPP_WARN(node_->get_logger(), "Received empty octree (no occupied leaves)");
    // No occupied leaves — set reasonable default small bounds around origin
    min_x = -1.0; min_y = -1.0; min_z = -1.0;
    max_x =  1.0; max_y =  1.0; max_z =  1.0;
    RCLCPP_WARN(node_->get_logger(), "No occupied leaves: using default small bounds for graph building.");
  }

  min_bound_ = {min_x, min_y, min_z};
  max_bound_ = {max_x, max_y, max_z};

  grid_size_x_ = static_cast<int>(std::ceil((max_x - min_x) / active_voxel_size_)) + 1;
  grid_size_y_ = static_cast<int>(std::ceil((max_y - min_y) / active_voxel_size_)) + 1;
  grid_size_z_ = static_cast<int>(std::ceil((max_z - min_z) / active_voxel_size_)) + 1;

  RCLCPP_DEBUG(node_->get_logger(), "Octomap update: frame=%s, resolution=%.3f, total_occupied=%zu, new_voxels=%zu",
              map_frame_.c_str(), octree_->getResolution(), occupied, new_voxels);

  // Mark graph dirty so background timer will update connectivity graph
  graph_dirty_ = true;
}

rcl_interfaces::msg::SetParametersResult AstarOctoPlanner::reconfigureCallback(std::vector<rclcpp::Parameter> parameters)
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
    } else if (parameter.get_name() == name_ + ".robot_width") {
      robot_width_ = parameter.as_double();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated robot_width to " << robot_width_);
    } else if (parameter.get_name() == name_ + ".publish_graph_markers") {
      bool newval = parameter.as_bool();
      if (newval != publish_graph_markers_) {
        publish_graph_markers_ = newval;
        RCLCPP_INFO_STREAM(node_->get_logger(), "Updated publish_graph_markers to " << publish_graph_markers_);
        // if enabling now and graph is already built, immediately publish
        if (publish_graph_markers_) {
          std::lock_guard<std::mutex> lock(graph_mutex_);
          publishGraphMarkers();
        }
      }
    } else if (parameter.get_name() == name_ + ".robot_length") {
      robot_length_ = parameter.as_double();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated robot_length to " << robot_length_);
    } else if (parameter.get_name() == name_ + ".robot_height") {
      robot_height_ = parameter.as_double();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated robot_height to " << robot_height_);
    } else if (parameter.get_name() == name_ + ".footprint_margin") {
      footprint_margin_ = parameter.as_double();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated footprint_margin to " << footprint_margin_);
    } else if (parameter.get_name() == name_ + ".footprint_samples_x") {
      footprint_samples_x_ = parameter.as_int();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated footprint_samples_x to " << footprint_samples_x_);
    } else if (parameter.get_name() == name_ + ".footprint_samples_y") {
      footprint_samples_y_ = parameter.as_int();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated footprint_samples_y to " << footprint_samples_y_);
    } else if (parameter.get_name() == name_ + ".max_surface_distance") {
      max_surface_distance_ = parameter.as_double();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated max_surface_distance to " << max_surface_distance_);
    } else if (parameter.get_name() == name_ + ".max_step_height") {
      max_step_height_ = parameter.as_double();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated max_step_height to " << max_step_height_);
    } else if (parameter.get_name() == name_ + ".enable_stair_edges") {
      enable_stair_edges_ = parameter.as_bool();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated enable_stair_edges to " << enable_stair_edges_);
    } else if (parameter.get_name() == name_ + ".stair_xy_radius") {
      stair_xy_radius_ = parameter.as_double();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated stair_xy_radius to " << stair_xy_radius_);
    } else if (parameter.get_name() == name_ + ".stair_vertical_margin") {
      stair_vertical_margin_ = parameter.as_double();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated stair_vertical_margin to " << stair_vertical_margin_);
    } else if (parameter.get_name() == name_ + ".min_vertical_clearance") {
      min_vertical_clearance_ = parameter.as_double();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated min_vertical_clearance to " << min_vertical_clearance_);
    } else if (parameter.get_name() == name_ + ".max_vertical_clearance") {
      max_vertical_clearance_ = parameter.as_double();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated max_vertical_clearance to " << max_vertical_clearance_);
    } else if (parameter.get_name() == name_ + ".enable_octomap_updates") {
      bool newval = parameter.as_bool();
      if (newval != enable_octomap_updates_) {
        enable_octomap_updates_ = newval;
        RCLCPP_INFO_STREAM(node_->get_logger(), "Updated enable_octomap_updates to " << enable_octomap_updates_);
        if (!enable_octomap_updates_) {
          // stop auto-rebuilding; keep existing graph
          graph_dirty_ = false;
        }
      }
    } else if (parameter.get_name() == name_ + ".octomap_topic") {
      std::string new_topic = parameter.as_string();
      if (new_topic != octomap_topic_) {
        RCLCPP_INFO_STREAM(node_->get_logger(), "octomap_topic changed from '" << octomap_topic_ << "' to '" << new_topic << "' -> recreating subscription.");
        octomap_topic_ = new_topic;
        // recreate subscription on new topic
        try {
          octomap_sub_.reset();
          octomap_sub_ = node_->create_subscription<octomap_msgs::msg::Octomap>(
            octomap_topic_, 1,
            std::bind(&AstarOctoPlanner::octomapCallback, this, std::placeholders::_1));
        } catch (const std::exception &e) {
          RCLCPP_ERROR(node_->get_logger(), "Failed to recreate octomap subscription on '%s': %s", octomap_topic_.c_str(), e.what());
        }
      }
    } else if (parameter.get_name() == name_ + ".penalty_spread_radius") {
      penalty_spread_radius_ = parameter.as_double();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated penalty_spread_radius to " << penalty_spread_radius_);
    } else if (parameter.get_name() == name_ + ".penalty_spread_factor") {
      penalty_spread_factor_ = parameter.as_double();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Updated penalty_spread_factor to " << penalty_spread_factor_);
    } else if (parameter.get_name() == name_ + ".worker_thread_limit") {
      int new_limit = parameter.as_int();
      if (new_limit < 0) {
        RCLCPP_WARN(node_->get_logger(), "worker_thread_limit cannot be negative (requested %d). Keeping %d.", new_limit, worker_thread_limit_);
      } else {
        worker_thread_limit_ = new_limit;
        RCLCPP_INFO(node_->get_logger(), "Updated worker_thread_limit to %d", worker_thread_limit_);
      }
    } else if (parameter.get_name() == name_ + ".incremental_graph_build") {
      bool newval = parameter.as_bool();
      if (newval != incremental_graph_build_) {
        incremental_graph_build_ = newval;
        RCLCPP_INFO_STREAM(node_->get_logger(), "Updated incremental_graph_build to " << incremental_graph_build_);
        if (!incremental_graph_build_) {
          // Switching to full rebuild mode: clear tracking sets and mark graph dirty
          // so the next update does a full rebuild
          processed_occupied_keys_.clear();
          nodes_needing_adjacency_update_.clear();
          graph_dirty_ = true;
          RCLCPP_INFO(node_->get_logger(), "Cleared incremental tracking state. Next update will do a full rebuild.");
        }
      }
    }
  }
  result.successful = true;
  return result;
}

} // namespace astar_octo_planner

// The following legacy helpers were removed from the source as they are no longer used
// by the graph-based planner implementation. Their declarations were already removed
// from the public header.
//
// hasNoOccupiedCellsAbove: removed (legacy helper not used)
