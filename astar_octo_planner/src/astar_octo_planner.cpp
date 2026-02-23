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
#include <std_msgs/msg/color_rgba.hpp>
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

    // --- Double-buffered graph access ---
    // Get a reference to the active graph. This is a shared_ptr copy, so the
    // graph remains valid for the duration of this planning call even if the
    // background thread swaps in a new graph.
    std::shared_ptr<GraphData> planning_graph;
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      planning_graph = active_graph_;
    }

    // If no active graph exists yet, we must build one synchronously (first call only).
    // After that, the background thread will keep updating and we'll never block here.
    if (!planning_graph || planning_graph->empty()) {
      RCLCPP_INFO(node_->get_logger(), "No active graph available — building connectivity graph synchronously (first time)...");
      auto new_graph = std::make_shared<GraphData>();
      buildConnectivityGraphInto(new_graph);
      {
        std::lock_guard<std::mutex> lock(graph_mutex_);
        active_graph_ = new_graph;
        // Also update legacy members for marker publishing until fully migrated
        graph_nodes_ = new_graph->nodes;
        graph_adj_ = new_graph->adj;
        graph_node_penalty_ = new_graph->node_penalty;
        graph_penalized_nodes_ = new_graph->penalized_nodes;
        graph_dirty_ = false;
      }
      planning_graph = new_graph;
      // Penalties will be computed on-demand below (incremental check)
      RCLCPP_INFO(node_->get_logger(), "Graph built: nodes=%zu", planning_graph->size());
    } else if (graph_dirty_) {
      // Graph is dirty but we have one to use. Log and proceed (background will update).
      RCLCPP_DEBUG(node_->get_logger(), "Graph is dirty but using existing graph (nodes=%zu). Background rebuild in progress.",
                   planning_graph->size());
    }

    // --- On-demand penalty/costmap computation ---
    // Always recompute all penalties from scratch so every node in the graph
    // is guaranteed to have a penalty value in the costmap.
    if (planning_graph && !planning_graph->nodes.empty()) {
      planning_graph->node_penalty.clear();
      planning_graph->penalized_nodes.clear();

      {
        RCLCPP_INFO(node_->get_logger(), "Computing penalties for %zu nodes (full recompute)...",
                    planning_graph->nodes.size());
        auto t_pen_start = std::chrono::steady_clock::now();

        // ================================================================
        // DUAL penalty detection:
        //
        // 1) CENTROID-SHIFT (Ahmed et al.) — works great on flat floors
        //    with walls. Tight z_tol keeps floors separate. Gives smooth
        //    gradient from walls to floor center.
        //
        // 2) GRAPH-ADJACENCY BORDER — for ramp edges, cliffs, any plane
        //    border without walls. Uses the graph adjacency list which
        //    already correctly connects walkable nodes across slopes.
        //    Checks angular XY coverage of walkable graph neighbors:
        //    if incomplete → edge of walkable surface.
        //
        // Final penalty = max(centroid_shift_penalty, graph_border_penalty)
        // ================================================================

        // --- Spatial hash of walkable nodes for centroid-shift ---
        const double cell_size = active_voxel_size_ * 2.0;
        struct GridKey { int gx, gy; bool operator==(const GridKey &o) const { return gx==o.gx && gy==o.gy; } };
        struct GridHash { size_t operator()(const GridKey &k) const { return std::hash<int>()(k.gx) ^ (std::hash<int>()(k.gy) << 16); } };
        std::unordered_map<GridKey, std::vector<size_t>, GridHash> walkable_grid;

        struct WalkableEntry { std::string id; octomap::point3d center; double z; };
        std::vector<WalkableEntry> walkable_list;
        walkable_list.reserve(planning_graph->nodes.size());
        // Also build id→index map for graph adjacency lookup
        std::unordered_map<std::string, size_t> id_to_idx;
        for (const auto &kv : planning_graph->nodes) {
          if (!kv.second.is_walkable) continue;
          size_t idx = walkable_list.size();
          walkable_list.push_back({kv.first, kv.second.center, static_cast<double>(kv.second.center.z())});
          id_to_idx[kv.first] = idx;
          int gx = static_cast<int>(std::floor(kv.second.center.x() / cell_size));
          int gy = static_cast<int>(std::floor(kv.second.center.y() / cell_size));
          walkable_grid[{gx, gy}].push_back(idx);
        }

        // ---------- PART 1: Centroid-shift (flat floor + walls) ----------
        const double search_radius = std::max(penalty_spread_radius_, active_voxel_size_ * 3.0);
        const double z_tol = active_voxel_size_ * 1.0; // tight — flat surfaces only
        const int search_cells = static_cast<int>(std::ceil(search_radius / cell_size)) + 1;

        size_t penalized_count = 0;
        size_t edge_count = 0;
        double max_shift_seen = 0.0;

        struct ShiftInfo { double shift_xy = 0.0; double shift_x = 0.0; double shift_y = 0.0; int n_neighbors = 0; };
        std::vector<ShiftInfo> shift_data(walkable_list.size());

        for (size_t wi = 0; wi < walkable_list.size(); ++wi) {
          const auto &w = walkable_list[wi];
          int gx = static_cast<int>(std::floor(w.center.x() / cell_size));
          int gy = static_cast<int>(std::floor(w.center.y() / cell_size));

          double sum_x = 0.0, sum_y = 0.0;
          int count = 0;

          for (int dgx = -search_cells; dgx <= search_cells; ++dgx) {
            for (int dgy = -search_cells; dgy <= search_cells; ++dgy) {
              auto it = walkable_grid.find({gx+dgx, gy+dgy});
              if (it == walkable_grid.end()) continue;
              for (size_t nidx : it->second) {
                if (nidx == wi) continue;
                const auto &nb = walkable_list[nidx];
                if (std::abs(nb.z - w.z) > z_tol) continue; // same level only
                double dx = nb.center.x() - w.center.x();
                double dy = nb.center.y() - w.center.y();
                double dist = std::sqrt(dx*dx + dy*dy);
                if (dist <= search_radius) {
                  sum_x += nb.center.x();
                  sum_y += nb.center.y();
                  ++count;
                }
              }
            }
          }

          if (count >= 2) {
            double cx = sum_x / count;
            double cy = sum_y / count;
            double sx = cx - w.center.x();
            double sy = cy - w.center.y();
            double shift = std::sqrt(sx*sx + sy*sy);
            shift_data[wi] = {shift, sx, sy, count};
            max_shift_seen = std::max(max_shift_seen, shift);
          }
        }

        const double norm_factor = search_radius * 0.75;  // higher → less CS penalty on interior floor

        // ---------- PART 2: Compute both penalties per node ----------
        size_t graph_border_count = 0;
        std::vector<double> cs_penalties(walkable_list.size(), 0.0);
        std::vector<double> gb_penalties(walkable_list.size(), 0.0);

        for (size_t wi = 0; wi < walkable_list.size(); ++wi) {
          const auto &w = walkable_list[wi];
          const auto &si = shift_data[wi];

          // --- Centroid-shift penalty ---
          double cs_penalty = 0.0;
          if (si.n_neighbors >= 2 && si.shift_xy > 1e-6) {
            double ratio = std::min(1.0, si.shift_xy / norm_factor);
            cs_penalty = wall_penalty_weight_ * ratio * ratio;
            if (ratio > 0.3) ++edge_count;

            // Corner detection for centroid-shift
            if (ratio > 0.2 && si.n_neighbors >= 4) {
              int gx = static_cast<int>(std::floor(w.center.x() / cell_size));
              int gy = static_cast<int>(std::floor(w.center.y() / cell_size));
              double shift_angle = std::atan2(si.shift_y, si.shift_x);
              int quadrant_mask = 0;
              for (int dgx = -search_cells; dgx <= search_cells; ++dgx) {
                for (int dgy = -search_cells; dgy <= search_cells; ++dgy) {
                  auto it = walkable_grid.find({gx+dgx, gy+dgy});
                  if (it == walkable_grid.end()) continue;
                  for (size_t nidx : it->second) {
                    if (nidx == wi) continue;
                    const auto &nb = walkable_list[nidx];
                    if (std::abs(nb.z - w.z) > z_tol) continue;
                    double dx = nb.center.x() - w.center.x();
                    double dy = nb.center.y() - w.center.y();
                    if (dx*dx + dy*dy > search_radius * search_radius) continue;
                    double angle = std::atan2(dy, dx) - shift_angle;
                    while (angle > M_PI) angle -= 2.0 * M_PI;
                    while (angle < -M_PI) angle += 2.0 * M_PI;
                    int q;
                    if (angle >= -M_PI/4 && angle < M_PI/4) q = 0;
                    else if (angle >= M_PI/4 && angle < 3*M_PI/4) q = 1;
                    else if (angle >= -3*M_PI/4 && angle < -M_PI/4) q = 3;
                    else q = 2;
                    quadrant_mask |= (1 << q);
                  }
                }
              }
              int n_quadrants = __builtin_popcount(quadrant_mask);
              if (n_quadrants <= 2 && ratio > 0.4) {
                double corner_factor = (3.0 - n_quadrants) / 2.0;
                cs_penalty += corner_penalty_weight_ * ratio * corner_factor;
              }
            }
          }

          // --- Graph-adjacency border penalty (handles ramps/cliffs) ---
          // Check which XY octants have walkable graph neighbors.
          // The graph adjacency was built by buildConnectivityGraph() which
          // connects nodes across height changes on ramps — no z_tol issue.
          double gb_penalty = 0.0;
          {
            auto adj_it = planning_graph->adj.find(w.id);
            int octant_mask = 0;
            int walkable_neighbor_count = 0;

            if (adj_it != planning_graph->adj.end()) {
              for (const auto &nb_id : adj_it->second) {
                auto nb_it = planning_graph->nodes.find(nb_id);
                if (nb_it == planning_graph->nodes.end() || !nb_it->second.is_walkable) continue;
                ++walkable_neighbor_count;
                double dx = nb_it->second.center.x() - w.center.x();
                double dy = nb_it->second.center.y() - w.center.y();
                double angle = std::atan2(dy, dx);
                // Map to octant 0-7
                int octant = static_cast<int>(std::floor((angle + M_PI) / (M_PI / 4.0))) % 8;
                octant_mask |= (1 << octant);
              }
            }

            int covered_octants = __builtin_popcount(octant_mask);
            // Full coverage = 8 octants → interior.
            // Missing octants → border of walkable surface.
            // This works on ramps because the graph connects across slopes.
            if (covered_octants < 7 && walkable_neighbor_count >= 1) {
              int missing = 8 - covered_octants;
              double border_ratio = static_cast<double>(missing) / 8.0;
              gb_penalty = wall_penalty_weight_ * border_ratio;

              // Corner: many missing octants → exposed edge or corner
              if (missing >= 4) {
                gb_penalty += corner_penalty_weight_ * 0.5 * border_ratio;
              }
              ++graph_border_count;
            } else if (walkable_neighbor_count == 0) {
              // Isolated node → high penalty
              gb_penalty = wall_penalty_weight_;
            }
          }

          cs_penalties[wi] = cs_penalty;
          gb_penalties[wi] = gb_penalty;
        }

        // ---------- PART 3: Spread graph-border penalty through graph edges ----------
        // Multi-source Dijkstra from border nodes; penalty decays linearly
        // with graph-distance so ramp edges get a smooth cost gradient.
        const double spread_radius = penalty_spread_radius_;   // 0.60m
        const double spread_decay  = penalty_spread_factor_;   // 0.25
        std::vector<double> spread_gb(walkable_list.size(), 0.0);
        {
          struct SpreadEntry {
            double dist;
            size_t wi;
            double source_gb;
          };
          auto cmp = [](const SpreadEntry &a, const SpreadEntry &b) {
            return a.dist > b.dist;
          };
          std::priority_queue<SpreadEntry, std::vector<SpreadEntry>, decltype(cmp)> pq(cmp);
          std::vector<double> best_dist(walkable_list.size(), std::numeric_limits<double>::max());

          // Seed: every graph-border node
          for (size_t wi = 0; wi < walkable_list.size(); ++wi) {
            if (gb_penalties[wi] > 1e-9) {
              spread_gb[wi] = gb_penalties[wi];  // keep full penalty at source
              best_dist[wi] = 0.0;
              pq.push({0.0, wi, gb_penalties[wi]});
            }
          }

          while (!pq.empty()) {
            auto top = pq.top();
            pq.pop();
            if (top.dist > best_dist[top.wi] + 1e-9) continue;

            auto adj_it = planning_graph->adj.find(walkable_list[top.wi].id);
            if (adj_it == planning_graph->adj.end()) continue;

            for (const auto &nb_id : adj_it->second) {
              auto idx_it = id_to_idx.find(nb_id);
              if (idx_it == id_to_idx.end()) continue;
              size_t nb_wi = idx_it->second;

              double edge_len = walkable_list[top.wi].center.distance(
                  walkable_list[nb_wi].center);
              double new_dist = top.dist + edge_len;
              if (new_dist >= spread_radius) continue;

              // Linear decay
              double decay = 1.0 - (new_dist / spread_radius);
              double new_val = top.source_gb * decay * spread_decay;

              if (new_val > spread_gb[nb_wi] + 1e-9) {
                spread_gb[nb_wi] = new_val;
                best_dist[nb_wi] = new_dist;
                pq.push({new_dist, nb_wi, top.source_gb});
              }
            }
          }
        }

        // ---------- PART 4: Final penalty = max(cs, gb, spread_gb) ----------
        for (size_t wi = 0; wi < walkable_list.size(); ++wi) {
          double penalty = std::max({cs_penalties[wi], gb_penalties[wi], spread_gb[wi]});
          planning_graph->node_penalty[walkable_list[wi].id] = penalty;
          if (penalty > 1e-9) {
            planning_graph->penalized_nodes.insert(walkable_list[wi].id);
            ++penalized_count;
          }
        }

        // Non-walkable nodes: zero penalty
        for (const auto &kv : planning_graph->nodes) {
          if (!kv.second.is_walkable) {
            planning_graph->node_penalty[kv.first] = 0.0;
          }
        }

        auto t_pen_end = std::chrono::steady_clock::now();
        double dur_pen = std::chrono::duration_cast<std::chrono::duration<double>>(t_pen_end - t_pen_start).count();
        RCLCPP_INFO(node_->get_logger(),
                    "Penalty: %.3f s, walkable=%zu, cs_edges=%zu, graph_borders=%zu, penalized=%zu (search_r=%.2fm)",
                    dur_pen, walkable_list.size(), edge_count, graph_border_count, penalized_count, search_radius);

        // Publish penalty markers now that they're computed
        if (publish_graph_markers_ && graph_marker_pub_) {
          publishGraphMarkers(planning_graph);
        }
      }  // end penalty computation block

      penalties_dirty_ = false;
    }

    // Find closest graph nodes to start and goal using the planning_graph
    std::string start_id = findClosestGraphNode(octomap::point3d(start_world.x, start_world.y, start_world.z), planning_graph);
    std::string goal_id  = findClosestGraphNode(octomap::point3d(goal_world.x,  goal_world.y,  goal_world.z), planning_graph);

    // Validate start node has walkable neighbors — if not, re-snap to a connected node
    if (!start_id.empty()) {
      auto adj_it = planning_graph->adj.find(start_id);
      bool has_walkable_neighbor = false;
      if (adj_it != planning_graph->adj.end()) {
        for (const auto &nb : adj_it->second) {
          auto nb_it = planning_graph->nodes.find(nb);
          if (nb_it != planning_graph->nodes.end() && nb_it->second.is_walkable) {
            has_walkable_neighbor = true;
            break;
          }
        }
      }
      if (!has_walkable_neighbor) {
        RCLCPP_WARN(node_->get_logger(),
          "Start node %s has no walkable neighbors — searching for a connected start node...", start_id.c_str());
        // Find the nearest walkable node that has at least 1 walkable neighbor
        octomap::point3d start_pt(start_world.x, start_world.y, start_world.z);
        double best_score = std::numeric_limits<double>::infinity();
        std::string best_connected_id;
        for (const auto &kv : planning_graph->nodes) {
          if (!kv.second.is_walkable) continue;
          // Check this node has at least 1 walkable neighbor
          auto a_it = planning_graph->adj.find(kv.first);
          if (a_it == planning_graph->adj.end() || a_it->second.empty()) continue;
          bool connected = false;
          for (const auto &nb : a_it->second) {
            auto nit = planning_graph->nodes.find(nb);
            if (nit != planning_graph->nodes.end() && nit->second.is_walkable) { connected = true; break; }
          }
          if (!connected) continue;
          double dx = kv.second.center.x() - start_pt.x();
          double dy = kv.second.center.y() - start_pt.y();
          double dz = kv.second.center.z() - start_pt.z();
          double score = dx*dx + dy*dy + dz*dz;
          if (score < best_score) { best_score = score; best_connected_id = kv.first; }
        }
        if (!best_connected_id.empty()) {
          const auto &cn = planning_graph->nodes.at(best_connected_id);
          RCLCPP_WARN(node_->get_logger(),
            "Re-snapped start to connected node %s (%.3f, %.3f, %.3f) dist=%.3f m",
            best_connected_id.c_str(), cn.center.x(), cn.center.y(), cn.center.z(),
            std::sqrt(best_score));
          start_id = best_connected_id;
        }
      }
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
      const auto &s = planning_graph->nodes.at(start_id);
      RCLCPP_INFO(node_->get_logger(), "Start graph center: (%.3f, %.3f, %.3f)", s.center.x(), s.center.y(), s.center.z());
      centers_ok &= valid_center(s.center);
    }
    if (!goal_id.empty()) {
      const auto &g = planning_graph->nodes.at(goal_id);
      RCLCPP_INFO(node_->get_logger(), "Goal graph center: (%.3f, %.3f, %.3f)", g.center.x(), g.center.y(), g.center.z());
      centers_ok &= valid_center(g.center);
    }

    if (!centers_ok || start_id.empty() || goal_id.empty()) {
      RCLCPP_WARN(node_->get_logger(), "Graph path not available or invalid centers. Aborting plan (NO_PATH_FOUND) for debugging.");
      message = "Graph path not available or invalid centers";
      return mbf_msgs::action::GetPath::Result::NO_PATH_FOUND;
    } else {
      // Use the planning_graph directly - it's a shared_ptr copy that won't change
      // during this planning call, even if background thread swaps the active graph.
      const auto& local_nodes = planning_graph->nodes;
      const auto& local_adj = planning_graph->adj;
      const auto& local_node_penalty = planning_graph->node_penalty;
      const auto& local_penalized_nodes = planning_graph->penalized_nodes;

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
          std::unordered_map<std::string,double> node_penalty(local_node_penalty.begin(), local_node_penalty.end());
          std::unordered_set<std::string> penalized_nodes(local_penalized_nodes.begin(), local_penalized_nodes.end());

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
              // Skip non-walkable nodes (walls, narrow ledges) — never route through them
              if (!local_nodes.at(nb).is_walkable) continue;
              // Edge collision check: raycast between nodes at robot body height
              // to prevent paths that cut through walls between two walkable floors
              if (!isEdgeCollisionFree(local_nodes.at(cur.id).center, local_nodes.at(nb).center)) continue;
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
        RCLCPP_WARN(node_->get_logger(), "No full path found on graph between %s and %s", start_id.c_str(), goal_id.c_str());

        // --- Build partial path to the expanded node closest to the goal ---
        // This lets the user see exactly where the planner gets stuck.
        std::string closest_to_goal_id;
        double closest_dist = std::numeric_limits<double>::infinity();
        const auto& goal_node = local_nodes.at(goal_id);
        for (const auto& nid : expanded_set) {
          auto it_n = local_nodes.find(nid);
          if (it_n == local_nodes.end()) continue;
          double dx = it_n->second.center.x() - goal_node.center.x();
          double dy = it_n->second.center.y() - goal_node.center.y();
          double dz = it_n->second.center.z() - goal_node.center.z();
          double d = dx*dx + dy*dy + dz*dz;
          if (d < closest_dist) { closest_dist = d; closest_to_goal_id = nid; }
        }

        if (!closest_to_goal_id.empty()) {
          closest_dist = std::sqrt(closest_dist);
          const auto& closest_node = local_nodes.at(closest_to_goal_id);
          RCLCPP_WARN(node_->get_logger(),
            "Partial path: closest expanded node to goal is %s (%.3f, %.3f, %.3f), dist=%.3f m",
            closest_to_goal_id.c_str(),
            closest_node.center.x(), closest_node.center.y(), closest_node.center.z(),
            closest_dist);

          // Reconstruct partial path from start to closest-to-goal node
          std::vector<std::string> partial_ids;
          std::string u = closest_to_goal_id;
          while (came_from.find(u) != came_from.end()) { partial_ids.push_back(u); u = came_from.at(u); }
          partial_ids.push_back(start_id);
          std::reverse(partial_ids.begin(), partial_ids.end());

          // Publish partial path for visualization (so user sees where planner gets stuck)
          nav_msgs::msg::Path partial_msg;
          partial_msg.header.stamp = node_->now();
          partial_msg.header.frame_id = path_frame;
          for (const auto &pid : partial_ids) {
            const auto &gn = local_nodes.at(pid);
            geometry_msgs::msg::PoseStamped ps;
            ps.header.frame_id = path_frame;
            ps.header.stamp = now;
            ps.pose.position.x = gn.center.x();
            ps.pose.position.y = gn.center.y();
            ps.pose.position.z = gn.center.z();
            ps.pose.orientation.w = 1.0;
            partial_msg.poses.push_back(ps);
          }
          path_pub_->publish(partial_msg);

          // Publish lifted copy too
          nav_msgs::msg::Path lifted_partial = partial_msg;
          for (auto &p : lifted_partial.poses) { p.pose.position.z += 0.5; }
          body_height_path_pub_->publish(lifted_partial);

          RCLCPP_WARN(node_->get_logger(),
            "Published partial path (%zu waypoints) to closest reachable node (%.3f m from goal)",
            partial_ids.size(), closest_dist);
        }

        // --- Connected-component diagnostic ---
        size_t start_component_size = expanded_set.size();

        // BFS from goal (walkable nodes only) to find its component
        std::unordered_set<std::string> goal_component;
        {
          std::queue<std::string> bfs_q;
          bfs_q.push(goal_id);
          goal_component.insert(goal_id);
          while (!bfs_q.empty()) {
            std::string u = bfs_q.front(); bfs_q.pop();
            auto it_a = local_adj.find(u);
            if (it_a == local_adj.end()) continue;
            for (const auto &v : it_a->second) {
              if (local_nodes.find(v) == local_nodes.end()) continue;
              if (!local_nodes.at(v).is_walkable) continue; // Only walkable nodes
              if (goal_component.insert(v).second) bfs_q.push(v);
            }
          }
        }

        RCLCPP_WARN(node_->get_logger(),
          "DISCONNECTED GRAPH DIAGNOSTIC:\n"
          "  Total graph nodes: %zu\n"
          "  Start component (A* reachable): %zu nodes\n"
          "  Goal  component (walkable BFS): %zu nodes\n"
          "  Goal in start component: %s",
          local_nodes.size(),
          start_component_size,
          goal_component.size(),
          (expanded_set.count(goal_id) > 0) ? "YES" : "NO");

        // Log boundary nodes of start component (low neighbor count = edge of reachable area)
        size_t boundary_logged = 0;
        for (const auto &nid : expanded_set) {
          if (boundary_logged >= 5) break;
          auto it_a = local_adj.find(nid);
          size_t adj_count = (it_a != local_adj.end()) ? it_a->second.size() : 0;
          if (adj_count <= 3) {
            const auto &nd = local_nodes.at(nid);
            RCLCPP_WARN(node_->get_logger(),
              "  Boundary node: %s (%.3f, %.3f, %.3f) adj=%zu",
              nid.c_str(), nd.center.x(), nd.center.y(), nd.center.z(), adj_count);
            ++boundary_logged;
          }
        }

        // --- Bridge planning: find closest pair between components and stitch paths ---
        // The A* start component uses isEdgeCollisionFree (strict), and the goal BFS uses
        // walkable adjacency. Nodes between them may be in neither set. So we search ALL
        // walkable nodes in the graph to find the closest pair that spans the gap.
        //
        // Strategy: Run a walkable-only BFS from start (ignoring isEdgeCollisionFree) to
        // get the true start walkable component, then bridge from that to the goal component.
        std::unordered_set<std::string> start_walkable_component;
        {
          std::queue<std::string> bfs_q;
          bfs_q.push(start_id);
          start_walkable_component.insert(start_id);
          while (!bfs_q.empty()) {
            std::string u = bfs_q.front(); bfs_q.pop();
            auto it_a = local_adj.find(u);
            if (it_a == local_adj.end()) continue;
            for (const auto &v : it_a->second) {
              if (local_nodes.find(v) == local_nodes.end()) continue;
              if (!local_nodes.at(v).is_walkable) continue;
              if (start_walkable_component.insert(v).second) bfs_q.push(v);
            }
          }
        }

        RCLCPP_INFO(node_->get_logger(),
          "Bridge search: start walkable BFS=%zu nodes (A* expanded=%zu), goal walkable BFS=%zu nodes",
          start_walkable_component.size(), expanded_set.size(), goal_component.size());

        std::string bridge_start_id, bridge_goal_id;
        double bridge_dist_sq = std::numeric_limits<double>::infinity();

        // Collect walkable nodes EXCLUSIVE to each component for the bridge search
        std::vector<std::pair<std::string, octomap::point3d>> start_walkable_pts;
        std::vector<std::pair<std::string, octomap::point3d>> goal_walkable_pts;
        for (const auto &nid : start_walkable_component) {
          if (goal_component.count(nid)) continue;
          auto it_n = local_nodes.find(nid);
          if (it_n != local_nodes.end() && it_n->second.is_walkable) {
            start_walkable_pts.emplace_back(nid, it_n->second.center);
          }
        }
        for (const auto &nid : goal_component) {
          if (start_walkable_component.count(nid)) continue;
          auto it_n = local_nodes.find(nid);
          if (it_n != local_nodes.end() && it_n->second.is_walkable) {
            goal_walkable_pts.emplace_back(nid, it_n->second.center);
          }
        }

        RCLCPP_INFO(node_->get_logger(),
          "Bridge candidates: start_pts=%zu, goal_pts=%zu",
          start_walkable_pts.size(), goal_walkable_pts.size());

        // Find closest pair
        for (const auto &[sid, spt] : start_walkable_pts) {
          for (const auto &[gid, gpt] : goal_walkable_pts) {
            double dx = spt.x() - gpt.x();
            double dy = spt.y() - gpt.y();
            double dz = spt.z() - gpt.z();
            double d2 = dx*dx + dy*dy + dz*dz;
            if (d2 < bridge_dist_sq) {
              bridge_dist_sq = d2;
              bridge_start_id = sid;
              bridge_goal_id = gid;
            }
          }
        }

        double bridge_dist = std::sqrt(bridge_dist_sq);
        const double max_bridge_dist = 0.3; // Only bridge small gaps (up to 30 cm)

        if (!bridge_start_id.empty() && !bridge_goal_id.empty() && bridge_dist <= max_bridge_dist) {
          RCLCPP_WARN(node_->get_logger(),
            "BRIDGE PLANNING: bridging %.3f m gap between %s and %s",
            bridge_dist, bridge_start_id.c_str(), bridge_goal_id.c_str());

          // Run A* within start walkable component: start → bridge_start
          // (using walkable adjacency, no isEdgeCollisionFree, to avoid the disconnect)
          std::vector<std::string> path_to_bridge;
          {
            std::unordered_map<std::string, double> gs1;
            std::unordered_map<std::string, std::string> cf1;
            std::priority_queue<PQItem, std::vector<PQItem>, PQCmp> oq1;

            auto h_bridge_start = [&](const std::string &a) -> double {
              auto it = local_nodes.find(a);
              auto it2 = local_nodes.find(bridge_start_id);
              if (it == local_nodes.end() || it2 == local_nodes.end()) return 0.0;
              return it->second.center.distance(it2->second.center);
            };

            oq1.push({start_id, h_bridge_start(start_id), 0.0});
            gs1[start_id] = 0.0;

            while (!oq1.empty()) {
              PQItem cur = oq1.top(); oq1.pop();
              if (cur.id == bridge_start_id) {
                std::string u = bridge_start_id;
                while (cf1.find(u) != cf1.end()) { path_to_bridge.push_back(u); u = cf1.at(u); }
                path_to_bridge.push_back(start_id);
                std::reverse(path_to_bridge.begin(), path_to_bridge.end());
                break;
              }
              if (gs1.find(cur.id) != gs1.end() && cur.g > gs1.at(cur.id)) continue;
              if (start_walkable_component.count(cur.id) == 0) continue;
              auto it_a1 = local_adj.find(cur.id);
              if (it_a1 == local_adj.end()) continue;
              for (const auto &nb : it_a1->second) {
                if (local_nodes.find(nb) == local_nodes.end()) continue;
                if (!local_nodes.at(nb).is_walkable) continue;
                if (start_walkable_component.count(nb) == 0) continue;
                double mc = local_nodes.at(cur.id).center.distance(local_nodes.at(nb).center);
                double pen = getPrecomputedPenalty(nb);
                double tent = cur.g + mc + pen;
                if (gs1.find(nb) == gs1.end() || tent < gs1[nb]) {
                  cf1[nb] = cur.id;
                  gs1[nb] = tent;
                  oq1.push({nb, tent + h_bridge_start(nb), tent});
                }
              }
            }
          }

          // Run A* within goal component: bridge_goal → goal
          // (using raw adjacency — no isEdgeCollisionFree — to avoid re-creating the disconnect)
          std::vector<std::string> path_from_bridge;
          {
            std::unordered_map<std::string, double> gs2;
            std::unordered_map<std::string, std::string> cf2;
            std::priority_queue<PQItem, std::vector<PQItem>, PQCmp> oq2;

            oq2.push({bridge_goal_id, heuristic_local(bridge_goal_id), 0.0});
            gs2[bridge_goal_id] = 0.0;

            while (!oq2.empty()) {
              PQItem cur = oq2.top(); oq2.pop();
              if (cur.id == goal_id) {
                // Reconstruct
                std::string u = goal_id;
                while (cf2.find(u) != cf2.end()) { path_from_bridge.push_back(u); u = cf2.at(u); }
                path_from_bridge.push_back(bridge_goal_id);
                std::reverse(path_from_bridge.begin(), path_from_bridge.end());
                break;
              }
              if (gs2.find(cur.id) != gs2.end() && cur.g > gs2.at(cur.id)) continue;
              // Only expand within the goal component
              if (goal_component.count(cur.id) == 0) continue;
              auto it_a2 = local_adj.find(cur.id);
              if (it_a2 == local_adj.end()) continue;
              for (const auto &nb : it_a2->second) {
                if (local_nodes.find(nb) == local_nodes.end()) continue;
                if (!local_nodes.at(nb).is_walkable) continue;
                if (goal_component.count(nb) == 0) continue;
                double mc = local_nodes.at(cur.id).center.distance(local_nodes.at(nb).center);
                double pen = getPrecomputedPenalty(nb);
                double tent = cur.g + mc + pen;
                if (gs2.find(nb) == gs2.end() || tent < gs2[nb]) {
                  cf2[nb] = cur.id;
                  gs2[nb] = tent;
                  oq2.push({nb, tent + heuristic_local(nb), tent});
                }
              }
            }
          }

          if (!path_to_bridge.empty() && !path_from_bridge.empty()) {
            RCLCPP_INFO(node_->get_logger(),
              "BRIDGE PATH: start→bridge=%zu nodes, bridge gap=%.3f m, bridge→goal=%zu nodes",
              path_to_bridge.size(), bridge_dist, path_from_bridge.size());

            // Stitch the full path: path_to_bridge + bridge_goal_path
            path_ids.clear();
            for (const auto &pid : path_to_bridge) path_ids.push_back(pid);
            for (const auto &pid : path_from_bridge) path_ids.push_back(pid);

            RCLCPP_INFO(node_->get_logger(), "Bridge path total: %zu waypoints", path_ids.size());
          } else {
            if (path_to_bridge.empty()) {
              RCLCPP_WARN(node_->get_logger(),
                "Bridge planning failed: no path from start to bridge_start within start component");
            }
            if (path_from_bridge.empty()) {
              RCLCPP_WARN(node_->get_logger(),
                "Bridge planning failed: no path from bridge_goal to goal within goal component");
            }
          }
        } else if (!bridge_start_id.empty() && !bridge_goal_id.empty()) {
          RCLCPP_WARN(node_->get_logger(),
            "Bridge gap too large (%.3f m > %.3f m max) — not bridging", bridge_dist, max_bridge_dist);
        } else {
          RCLCPP_WARN(node_->get_logger(),
            "Bridge planning skipped: no candidate pairs found (start_pts=%zu, goal_pts=%zu)",
            start_walkable_pts.size(), goal_walkable_pts.size());
        }

        // If bridge planning succeeded, path_ids is non-empty and we fall through
        // to the normal path conversion below. Otherwise try a relaxed A*.

        // Relaxed A* fallback: if the start and goal are in the same walkable
        // component (BFS-reachable without isEdgeCollisionFree), the disconnect
        // is caused by overly-strict edge collision checks. Run A* using only
        // walkable adjacency (no isEdgeCollisionFree) to find a path.
        if (path_ids.empty() && start_walkable_component.count(goal_id) > 0) {
          RCLCPP_WARN(node_->get_logger(),
            "Components are identical (%zu walkable nodes). Running relaxed A* (no edge collision check)...",
            start_walkable_component.size());

          std::unordered_map<std::string, double> gs_relaxed;
          std::unordered_map<std::string, std::string> cf_relaxed;
          std::priority_queue<PQItem, std::vector<PQItem>, PQCmp> oq_relaxed;

          oq_relaxed.push({start_id, heuristic_local(start_id), 0.0});
          gs_relaxed[start_id] = 0.0;

          while (!oq_relaxed.empty()) {
            PQItem cur = oq_relaxed.top(); oq_relaxed.pop();
            if (cur.id == goal_id) {
              std::string u = goal_id;
              while (cf_relaxed.find(u) != cf_relaxed.end()) {
                path_ids.push_back(u); u = cf_relaxed.at(u);
              }
              path_ids.push_back(start_id);
              std::reverse(path_ids.begin(), path_ids.end());
              break;
            }
            if (gs_relaxed.find(cur.id) != gs_relaxed.end() && cur.g > gs_relaxed.at(cur.id)) continue;
            auto it_a = local_adj.find(cur.id);
            if (it_a == local_adj.end()) continue;
            for (const auto &nb : it_a->second) {
              if (local_nodes.find(nb) == local_nodes.end()) continue;
              if (!local_nodes.at(nb).is_walkable) continue;
              // NO isEdgeCollisionFree check — rely on walkable adjacency + penalties
              double mc = local_nodes.at(cur.id).center.distance(local_nodes.at(nb).center);
              double pen = getPrecomputedPenalty(nb);
              double tent = cur.g + mc + pen;
              if (gs_relaxed.find(nb) == gs_relaxed.end() || tent < gs_relaxed[nb]) {
                cf_relaxed[nb] = cur.id;
                gs_relaxed[nb] = tent;
                oq_relaxed.push({nb, tent + heuristic_local(nb), tent});
              }
            }
          }

          if (!path_ids.empty()) {
            RCLCPP_INFO(node_->get_logger(),
              "Relaxed A* found path: %zu waypoints", path_ids.size());
          } else {
            RCLCPP_WARN(node_->get_logger(),
              "Relaxed A* also failed to find path");
          }
        }

        if (path_ids.empty()) {
          message = "No path found on graph";
          return mbf_msgs::action::GetPath::Result::NO_PATH_FOUND;
        }
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
          // color ramp: low green -> high red (normalized to wall_penalty_weight_)
          std_msgs::msg::ColorRGBA c;
          double v = std::min(1.0, pen / std::max(1e-6, wall_penalty_weight_));
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

  // Publish a Z-lifted copy for visualization (not hidden inside voxels)
  nav_msgs::msg::Path lifted_msg = path_msg;
  for (auto &p : lifted_msg.poses) {
    p.pose.position.z += 0.5;
  }
  body_height_path_pub_->publish(lifted_msg);

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

  //Build graph from ALL occupied leafs (floor surfaces + walls)
  for (auto it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it) {
    ++total_leafs;
    if (!octree_->isNodeOccupied(*it)) continue;
    ++free_leafs; // reusing counter for occupied samples here
    octomap::point3d occ = it.getCoordinate();
    // Use actual voxel center (no nudge)
    double node_z = occ.z();
    if (!std::isfinite(occ.x()) || !std::isfinite(occ.y()) || !std::isfinite(node_z)) { ++skipped_nonfinite; continue; }
    if (occ.x() < min_bound_[0] - 1e-6 || occ.x() > max_bound_[0] + 1e-6 ||
        occ.y() < min_bound_[1] - 1e-6 || occ.y() > max_bound_[1] + 1e-6 ||
        node_z < min_bound_[2] - 1e-6 || node_z > max_bound_[2] + 1e-6) { ++skipped_out_of_bounds; continue; }

    // vertical clearance check above the voxel top surface to determine walkability
    bool vertical_clear = true;
    double stepz = std::max(active_voxel_size_, 0.05);
    double voxel_top = occ.z() + 0.5 * octree_->getNodeSize(it.getDepth());
    for (double z = voxel_top + 0.01; z <= voxel_top + robot_height_; z += stepz) {
      octomap::OcTreeNode* nn = octree_->search(occ.x(), occ.y(), z);
      if (nn && octree_->isNodeOccupied(nn)) { vertical_clear = false; break; }
    }
    // Count but don't skip non-walkable nodes — they're included for visualization
    if (!vertical_clear) { ++skipped_collision; }

    // Floor support check: verify the surface has enough horizontal extent
    // (rejects thin wall tops that pass vertical clearance)
    if (vertical_clear) {
      double node_sz = octree_->getNodeSize(it.getDepth());
      if (!hasFloorSupport(occ.x(), occ.y(), occ.z(), node_sz)) {
        vertical_clear = false; // Mark as non-walkable (wall top / narrow ledge)
      }
    }

    octomap::point3d center(occ.x(), occ.y(), node_z);
    octomap::OcTreeKey key = octree_->coordToKey(center);
    unsigned int depth = it.getDepth();
    double size = octree_->getNodeSize(depth);

    GraphNode gn;
    gn.key = key;
    gn.depth = depth;
    gn.center = center;
    gn.size = size;
    gn.is_walkable = vertical_clear;  // floor = true, wall = false

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

  std::vector<std::vector<std::string>> adjacency(node_snapshot.size());

  auto add_edge = [&](size_t from_idx, const std::string &neighbor_id) {
    if (from_idx >= adjacency.size()) return;
    auto &vec = adjacency[from_idx];
    if (std::find(vec.begin(), vec.end(), neighbor_id) == vec.end()) {
      vec.push_back(neighbor_id);
    }
  };

  // Simple spatial adjacency: connect nodes within ~1.5 voxel distance
  auto neighbor_worker = [&](size_t start, size_t end) {
    for (size_t idx = start; idx < end; ++idx) {
      const auto &id = node_snapshot[idx].first;
      const GraphNode &node = node_snapshot[idx].second;
      double max_dist = node.size * 1.8;  // ~1.8 voxel distance for diagonal neighbors
      double max_dist_sq = max_dist * max_dist;

      // Check all other nodes for proximity
      for (size_t j = 0; j < node_snapshot.size(); ++j) {
        if (j == idx) continue;
        const GraphNode &other = node_snapshot[j].second;
        double dx = other.center.x() - node.center.x();
        double dy = other.center.y() - node.center.y();
        double dz = other.center.z() - node.center.z();
        double dist_sq = dx*dx + dy*dy + dz*dz;
        if (dist_sq <= max_dist_sq) {
          add_edge(idx, node_snapshot[j].first);
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

  // Penalties are NOT computed here — they are computed on-demand when a path
  // is requested (in makePlan). This keeps the graph build fast and allows
  // incremental penalty computation for new nodes only.
  graph_node_penalty_.clear();
  graph_penalized_nodes_.clear();

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

void AstarOctoPlanner::buildConnectivityGraphInto(std::shared_ptr<GraphData>& target, double eps)
{
  // Build using the existing function (writes to class members as temp storage)
  buildConnectivityGraph(eps);

  // Copy structure results to the target GraphData (no penalties — computed on-demand)
  target->nodes = graph_nodes_;
  target->adj = graph_adj_;
  target->node_penalty.clear();
  target->penalized_nodes.clear();
  target->processed_occupied_keys = processed_occupied_keys_;
  target->nodes_needing_adjacency_update = nodes_needing_adjacency_update_;
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

  // Create graph nodes for each new occupied voxel (at actual voxel center position)
  std::vector<std::pair<std::string, GraphNode>> new_nodes;

  for (const auto& [occ, key, depth, node_size] : new_voxels) {
    // Use actual voxel center (no z-shift)
    double node_z = occ.z();

    if (!std::isfinite(occ.x()) || !std::isfinite(occ.y()) || !std::isfinite(node_z)) continue;
    if (occ.x() < min_bound_[0] - 1e-6 || occ.x() > max_bound_[0] + 1e-6 ||
        occ.y() < min_bound_[1] - 1e-6 || occ.y() > max_bound_[1] + 1e-6 ||
        node_z < min_bound_[2] - 1e-6 || node_z > max_bound_[2] + 1e-6) continue;

    // Vertical clearance check above voxel top surface
    bool vertical_clear = true;
    double stepz = std::max(active_voxel_size_, 0.05);
    double voxel_top = occ.z() + 0.5 * node_size;
    for (double z = voxel_top + 0.01; z <= voxel_top + robot_height_; z += stepz) {
      octomap::OcTreeNode* nn = octree_->search(occ.x(), occ.y(), z);
      if (nn && octree_->isNodeOccupied(nn)) { vertical_clear = false; break; }
    }
    // Include non-walkable voxels in graph for visualization (A* skips them)

    // Floor support check: verify the surface has enough horizontal extent
    if (vertical_clear) {
      if (!hasFloorSupport(occ.x(), occ.y(), occ.z(), node_size)) {
        vertical_clear = false; // wall top / narrow ledge
      }
    }

    octomap::point3d center(occ.x(), occ.y(), node_z);
    octomap::OcTreeKey center_key = octree_->coordToKey(center);

    GraphNode gn;
    gn.key = center_key;
    gn.depth = depth;
    gn.center = center;
    gn.size = node_size;
    gn.is_walkable = vertical_clear;  // floor = true, wall = false

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
  // Use a generous radius to avoid connectivity gaps between incremental batches
  double neighbor_search_radius = active_voxel_size_ * 5.0; // Search nearby existing nodes (increased from 3.0)
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

      // Match the full-build distance threshold (1.8x) to avoid connectivity gaps
      double thresh = node.size * 1.8;
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

    // --- Direct distance-based adjacency (octree-independent) ---
    // Mirror the full-build approach: connect to ANY graph node within 1.8x
    // voxel distance. This ensures connectivity even when the octree has
    // been cleared (e.g. by clearOccupiedCylinderAround) or has gaps.
    double max_dist_sq = (node.size * 1.8) * (node.size * 1.8);
    for (const auto& [cand_id, cand_node] : all_nodes_snapshot) {
      if (cand_id == node_id) continue;
      double dx = cand_node.center.x() - node.center.x();
      double dy = cand_node.center.y() - node.center.y();
      double dz = cand_node.center.z() - node.center.z();
      double dsq = dx*dx + dy*dy + dz*dz;
      if (dsq > max_dist_sq) continue;
      // Add edge if not already present (skip LOS check, matching full build)
      if (std::find(adj.begin(), adj.end(), cand_id) == adj.end()) {
        adj.push_back(cand_id);
      }
      auto& rev = graph_adj_[cand_id];
      if (std::find(rev.begin(), rev.end(), node_id) == rev.end()) {
        rev.push_back(node_id);
      }
    }
  }

  // Clear the pending update set
  nodes_needing_adjacency_update_.clear();

  // NOTE: Penalties are NOT computed here to save CPU. They will be computed
  // on-demand when a plan is requested (in makePlan). This avoids wasting
  // cycles recomputing the costmap every time the graph updates.

  // Publish updated markers (nodes only, no penalty markers since they're not computed yet)
  if (publish_graph_markers_) {
    publishGraphMarkers();
  }

  auto t_end = std::chrono::steady_clock::now();
  double dur = std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start).count();
  RCLCPP_INFO(node_->get_logger(), "Incremental graph update took %.3f s", dur);
}

void AstarOctoPlanner::updateConnectivityGraphIncrementalInto(std::shared_ptr<GraphData>& target, double eps)
{
  // Copy target's tracking state to class members (so incremental update can use them)
  processed_occupied_keys_ = target->processed_occupied_keys;
  nodes_needing_adjacency_update_ = target->nodes_needing_adjacency_update;
  graph_nodes_ = target->nodes;
  graph_adj_ = target->adj;
  // Don't copy penalties — they are not computed during graph build

  // Run the incremental update (writes to class members)
  updateConnectivityGraphIncremental(eps);

  // Copy structure results back to target (no penalties)
  target->nodes = graph_nodes_;
  target->adj = graph_adj_;
  target->processed_occupied_keys = processed_occupied_keys_;
  target->nodes_needing_adjacency_update = nodes_needing_adjacency_update_;
}

void AstarOctoPlanner::publishGraphMarkers()
{
  if (!graph_marker_pub_ || graph_nodes_.empty()) return;
  visualization_msgs::msg::MarkerArray ma;

  // All graph nodes marker (green)
  visualization_msgs::msg::Marker m;
  m.header.frame_id = map_frame_.empty() ? "map" : map_frame_;
  m.header.stamp = node_->now();
  m.ns = "graph_nodes";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::CUBE_LIST;
  m.action = visualization_msgs::msg::Marker::ADD;
  double scale = active_voxel_size_ > 0.0 ? std::max(active_voxel_size_, 0.05) : 0.05;
  m.scale.x = static_cast<float>(scale);
  m.scale.y = static_cast<float>(scale);
  m.scale.z = static_cast<float>(scale);
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
    // Color per node: green = walkable, red = non-walkable (wall/obstacle)
    std_msgs::msg::ColorRGBA c;
    if (kv.second.is_walkable) {
      c.r = 0.0f; c.g = 1.0f; c.b = 0.0f; c.a = 0.8f;
    } else {
      c.r = 1.0f; c.g = 0.0f; c.b = 0.0f; c.a = 0.8f;
    }
    m.colors.push_back(c);
  }
  ma.markers.push_back(m);
  graph_marker_pub_->publish(ma);
}

void AstarOctoPlanner::publishGraphMarkers(const std::shared_ptr<GraphData>& graph)
{
  if (!graph_marker_pub_ || !graph || graph->nodes.empty()) return;
  visualization_msgs::msg::MarkerArray ma;

  // All graph nodes marker — green=walkable, red=non-walkable
  visualization_msgs::msg::Marker m;
  m.header.frame_id = map_frame_.empty() ? "map" : map_frame_;
  m.header.stamp = node_->now();
  m.ns = "graph_nodes";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::CUBE_LIST;
  m.action = visualization_msgs::msg::Marker::ADD;
  double scale = active_voxel_size_ > 0.0 ? std::max(active_voxel_size_, 0.05) : 0.05;
  m.scale.x = static_cast<float>(scale);
  m.scale.y = static_cast<float>(scale);
  m.scale.z = static_cast<float>(scale);
  // Default color (overridden per point below)
  m.color.r = 0.0f;
  m.color.g = 1.0f;
  m.color.b = 0.0f;
  m.color.a = 0.8f;

  for (const auto &kv : graph->nodes) {
    geometry_msgs::msg::Point p;
    p.x = kv.second.center.x();
    p.y = kv.second.center.y();
    p.z = kv.second.center.z();
    m.points.push_back(p);
    // Color per node: green = walkable, red = non-walkable (wall/obstacle)
    std_msgs::msg::ColorRGBA c;
    if (kv.second.is_walkable) {
      c.r = 0.0f; c.g = 1.0f; c.b = 0.0f; c.a = 0.8f;
    } else {
      c.r = 1.0f; c.g = 0.0f; c.b = 0.0f; c.a = 0.8f;
    }
    m.colors.push_back(c);
  }
  ma.markers.push_back(m);

  // Penalty markers (red/green gradient based on penalty value)
  visualization_msgs::msg::Marker pen_m;
  pen_m.header.frame_id = map_frame_.empty() ? "map" : map_frame_;
  pen_m.header.stamp = node_->now();
  pen_m.ns = "graph_penalty";
  pen_m.id = 3;
  pen_m.type = visualization_msgs::msg::Marker::CUBE_LIST;
  pen_m.action = visualization_msgs::msg::Marker::ADD;
  pen_m.scale.x = static_cast<float>(scale);
  pen_m.scale.y = static_cast<float>(scale);
  pen_m.scale.z = static_cast<float>(scale);

  // Show ALL walkable nodes with green→red gradient based on penalty.
  // Zero-penalty nodes are rendered as green so there are no holes.
  for (const auto &kv : graph->nodes) {
    if (!kv.second.is_walkable) continue;
    geometry_msgs::msg::Point p;
    p.x = kv.second.center.x();
    p.y = kv.second.center.y();
    p.z = kv.second.center.z();
    pen_m.points.push_back(p);
    // Look up penalty (0 if not in map)
    double pen = 0.0;
    auto pit = graph->node_penalty.find(kv.first);
    if (pit != graph->node_penalty.end()) pen = pit->second;
    // color ramp: low green -> high red (normalized to wall_penalty_weight_)
    std_msgs::msg::ColorRGBA c;
    double v = std::min(1.0, pen / std::max(1e-6, wall_penalty_weight_));
    c.r = static_cast<float>(v);
    c.g = static_cast<float>(1.0 - v);
    c.b = 0.0f;
    c.a = 0.9f;
    pen_m.colors.push_back(c);
  }
  if (!pen_m.points.empty()) {
    ma.markers.push_back(pen_m);
  }

  graph_marker_pub_->publish(ma);
  RCLCPP_DEBUG(node_->get_logger(), "Published graph markers: nodes=%zu, penalty_nodes=%zu",
               m.points.size(), pen_m.points.size());
}

std::string AstarOctoPlanner::findClosestGraphNode(const octomap::point3d& p) const
{
  if (graph_nodes_.empty()) return std::string();
  double best = std::numeric_limits<double>::infinity();
  std::string best_id;
  // First pass: find candidates within XY radius, prefer lowest z (floor)
  const double xy_search_radius = 1.0;  // 1m XY search radius
  const double max_z_above_query = 0.3; // Only consider nodes up to 30cm above query point

  for (const auto &kv : graph_nodes_) {
    // Only consider walkable nodes for path planning
    if (!kv.second.is_walkable) continue;
    double dx = kv.second.center.x() - p.x();
    double dy = kv.second.center.y() - p.y();
    double dxy_sq = dx*dx + dy*dy;
    // Skip if too far in XY
    if (dxy_sq > xy_search_radius * xy_search_radius) continue;
    // Skip if node is above the query point (robot legs are above floor)
    double node_z = kv.second.center.z();
    if (node_z > p.z() + max_z_above_query) continue;
    // Score: prefer low z (floor), with small XY penalty
    double score = node_z + 0.1 * std::sqrt(dxy_sq);
    if (score < best) { best = score; best_id = kv.first; }
  }

  // Fallback: if no candidate found, use original 3D distance
  if (best_id.empty()) {
    best = std::numeric_limits<double>::infinity();
    for (const auto &kv : graph_nodes_) {
      if (!kv.second.is_walkable) continue;
      double dx = kv.second.center.x() - p.x();
      double dy = kv.second.center.y() - p.y();
      double dz = kv.second.center.z() - p.z();
      double d = dx*dx + dy*dy + dz*dz;
      if (d < best) { best = d; best_id = kv.first; }
    }
  }
  return best_id;
}

std::string AstarOctoPlanner::findClosestGraphNode(const octomap::point3d& p, const std::shared_ptr<GraphData>& graph) const
{
  if (!graph || graph->nodes.empty()) return std::string();
  double best = std::numeric_limits<double>::infinity();
  std::string best_id;
  // First pass: find candidates within XY radius, prefer lowest z (floor)
  const double xy_search_radius = 1.0;  // 1m XY search radius
  const double max_z_above_query = 0.3; // Only consider nodes up to 30cm above query point

  for (const auto &kv : graph->nodes) {
    // Only consider walkable nodes for path planning
    if (!kv.second.is_walkable) continue;
    double dx = kv.second.center.x() - p.x();
    double dy = kv.second.center.y() - p.y();
    double dxy_sq = dx*dx + dy*dy;
    // Skip if too far in XY
    if (dxy_sq > xy_search_radius * xy_search_radius) continue;
    // Skip if node is above the query point (robot legs are above floor)
    double node_z = kv.second.center.z();
    if (node_z > p.z() + max_z_above_query) continue;
    // Score: prefer low z (floor), with small XY penalty
    double score = node_z + 0.1 * std::sqrt(dxy_sq);
    if (score < best) { best = score; best_id = kv.first; }
  }

  // Fallback: if no candidate found, use original 3D distance
  if (best_id.empty()) {
    best = std::numeric_limits<double>::infinity();
    for (const auto &kv : graph->nodes) {
      if (!kv.second.is_walkable) continue;
      double dx = kv.second.center.x() - p.x();
      double dy = kv.second.center.y() - p.y();
      double dz = kv.second.center.z() - p.z();
      double d = dx*dx + dy*dy + dz*dz;
      if (d < best) { best = d; best_id = kv.first; }
    }
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

// Check horizontal floor support: sample a ring of directions at robot_radius_ distance
// and verify enough directions have an occupied neighbor at similar Z. This detects thin
// vertical structures (wall tops) that pass the vertical clearance check but are too
// narrow for the robot to stand on.
bool AstarOctoPlanner::hasFloorSupport(double x, double y, double z, double node_size) const
{
  if (!octree_) return false;

  const double search_radius = std::max(robot_radius_, node_size * 2.0);
  const double z_tol = std::max(active_voxel_size_ * 2.0, 0.15); // Allow Z variation for sloped surfaces / ramps
  const double step_z = std::max(active_voxel_size_, 0.05);
  const int num_dirs = std::max(4, floor_support_num_dirs_);
  int support_count = 0;

  for (int d = 0; d < num_dirs; ++d) {
    double angle = d * (2.0 * M_PI / num_dirs);
    double dx = search_radius * std::cos(angle);
    double dy = search_radius * std::sin(angle);

    // Check at the same Z and slightly above/below (handles ramps and uneven surfaces)
    bool found = false;
    for (double dz = -z_tol; dz <= z_tol; dz += step_z) {
      octomap::OcTreeNode* nn = octree_->search(x + dx, y + dy, z + dz);
      if (nn && octree_->isNodeOccupied(nn)) {
        found = true;
        break;
      }
    }
    if (found) ++support_count;
  }

  double ratio = static_cast<double>(support_count) / static_cast<double>(num_dirs);
  return ratio >= min_floor_support_ratio_;
}

// Edge collision check: sample points along the line from->to and at each sample
// check multiple Z heights (from above surface up to robot_height_) for occupied voxels.
// This prevents A* from connecting two walkable floor nodes across a wall.
bool AstarOctoPlanner::isEdgeCollisionFree(const octomap::point3d& from, const octomap::point3d& to) const
{
  if (!octree_) return true;

  double dist = from.distance(to);
  if (dist < 1e-6) return true;

  // Step along the edge roughly every half-voxel
  double step = std::max(active_voxel_size_ * 0.5, 0.05);
  int n_samples = std::max(2, static_cast<int>(std::ceil(dist / step)));

  // We check the free space ABOVE the surface where the robot body would be.
  // Surface nodes are occupied voxels, so start from above the voxel top.
  const double voxel_half = active_voxel_size_ * 0.5;
  const double z_start_offset = voxel_half + 0.05;  // just above voxel top surface
  const double z_step = std::max(active_voxel_size_, 0.1);
  const int n_z = std::max(2, static_cast<int>(std::ceil(robot_height_ / z_step)));

  for (int i = 1; i < n_samples; ++i) {  // skip endpoints (they're already validated as walkable)
    double t = static_cast<double>(i) / static_cast<double>(n_samples);
    double sx = from.x() + t * (to.x() - from.x());
    double sy = from.y() + t * (to.y() - from.y());
    double sz = from.z() + t * (to.z() - from.z());

    // Check from above surface voxel top up to robot_height_
    for (int iz = 0; iz <= n_z; ++iz) {
      double frac = static_cast<double>(iz) / static_cast<double>(n_z);
      double check_z = sz + z_start_offset + frac * robot_height_;
      octomap::OcTreeNode* nn = octree_->search(sx, sy, check_z);
      if (nn && octree_->isNodeOccupied(nn)) {
        return false;  // Wall or obstacle in the way
      }
    }
  }
  return true;
}

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
  body_height_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("~/body_height/path", rclcpp::QoS(1).transient_local());
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
  // Floor support parameters (horizontal extent check for wall rejection)
  min_floor_support_ratio_ = node_->declare_parameter(name_ + ".min_floor_support_ratio", min_floor_support_ratio_);
  floor_support_num_dirs_ = node_->declare_parameter(name_ + ".floor_support_num_dirs", floor_support_num_dirs_);
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

  // Create a periodic timer to (re)build the connectivity graph in background.
  // Uses double-buffering: build into a new graph, then swap pointers atomically.
  // This ensures makePlan() always has access to a valid graph (never blocks).
  graph_build_timer_ = node_->create_wall_timer(
    std::chrono::seconds(1),
    [this]() {
      if (!enable_octomap_updates_) return; // do nothing when updates disabled
      if (!graph_dirty_) return;
      // Avoid concurrent builds
      bool expected = false;
      if (!graph_building_.compare_exchange_strong(expected, true)) {
        //RCLCPP_DEBUG(node_->get_logger(), "Background graph build already in progress, skipping.");
        return;
      }

      // Build into a new graph (does NOT hold the mutex during the expensive work)
      auto new_graph = std::make_shared<GraphData>();

      // Copy incremental tracking state from the active graph (if any) so we can
      // do incremental updates. This is a brief lock just to copy the tracking sets.
      std::shared_ptr<GraphData> current_graph;
      {
        std::lock_guard<std::mutex> lock(graph_mutex_);
        current_graph = active_graph_;
      }

      // Use incremental update if we have an existing graph and incremental mode is enabled
      if (incremental_graph_build_ && current_graph && !current_graph->empty()) {
        // Start from a copy of the current graph for incremental update
        new_graph->nodes = current_graph->nodes;
        new_graph->adj = current_graph->adj;
        // Carry forward cached penalties so makePlan doesn't recompute them
        new_graph->node_penalty = current_graph->node_penalty;
        new_graph->penalized_nodes = current_graph->penalized_nodes;
        new_graph->processed_occupied_keys = current_graph->processed_occupied_keys;
        new_graph->nodes_needing_adjacency_update = current_graph->nodes_needing_adjacency_update;

        //RCLCPP_INFO(node_->get_logger(), "Background: incrementally updating connectivity graph...");
        updateConnectivityGraphIncrementalInto(new_graph);
        //RCLCPP_INFO(node_->get_logger(), "Background: connectivity graph updated. Nodes=%zu", new_graph->size());

        // Incremental update: keep existing penalties, only mark dirty for new nodes
        // Penalties will be computed on-demand only if params changed (not every update)
        // penalties_dirty_ is NOT set here — existing penalties remain valid
      } else {
        //RCLCPP_INFO(node_->get_logger(), "Background: rebuilding connectivity graph from scratch...");
        buildConnectivityGraphInto(new_graph);
        //RCLCPP_INFO(node_->get_logger(), "Background: connectivity graph ready. Nodes=%zu", new_graph->size());
        // Penalties are computed on-demand in makePlan, not during build
      }

      // Atomic swap: only hold mutex briefly to swap the pointer
      {
        std::lock_guard<std::mutex> lock(graph_mutex_);
        active_graph_ = new_graph;
        // Also update legacy members for marker publishing until fully migrated
        graph_nodes_ = new_graph->nodes;
        graph_adj_ = new_graph->adj;
        // Don't overwrite legacy penalty members — penalties live in active_graph_
        // and are computed on-demand in makePlan
        processed_occupied_keys_ = new_graph->processed_occupied_keys;
        nodes_needing_adjacency_update_ = new_graph->nodes_needing_adjacency_update;
        graph_dirty_ = false;
      }

      // Publish graph structure markers (green/red walkable nodes).
      // Penalty markers are published on-demand when a plan is requested.
      if (publish_graph_markers_) {
        publishGraphMarkers(new_graph);
      }

      graph_building_ = false;
      //RCLCPP_INFO(node_->get_logger(), "Background: graph swap complete, planning can use new graph immediately.");
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
  // Deserialize octomap message — auto-detect binary vs full format
  octomap::AbstractOcTree* tree_ptr = nullptr;
  if (msg->binary) {
    tree_ptr = octomap_msgs::binaryMsgToMap(*msg);
  } else {
    tree_ptr = octomap_msgs::fullMsgToMap(*msg);
  }
  if (!tree_ptr) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to convert Octomap message to AbstractOcTree (binary=%s)",
                 msg->binary ? "true" : "false");
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
        // Use actual voxel center (no z-shift)
        double node_z = cz;
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
    } else if (parameter.get_name() == name_ + ".corner_penalty_weight") {
      corner_penalty_weight_ = parameter.as_double();
      penalties_dirty_ = true; // Mark penalties dirty so they get recomputed on next plan
      RCLCPP_INFO(node_->get_logger(), "Updated corner_penalty_weight to %.1f (penalties will be recomputed on next plan)", corner_penalty_weight_);
    } else if (parameter.get_name() == name_ + ".centroid_penalty_weight") {
      centroid_penalty_weight_ = parameter.as_double();
      penalties_dirty_ = true; // Mark penalties dirty so they get recomputed on next plan
      RCLCPP_INFO(node_->get_logger(), "Updated centroid_penalty_weight to %.1f (penalties will be recomputed on next plan)", centroid_penalty_weight_);
    } else if (parameter.get_name() == name_ + ".ransac_radius") {
      ransac_radius_ = parameter.as_double();
      penalties_dirty_ = true;
      RCLCPP_INFO(node_->get_logger(), "Updated ransac_radius to %.2f (penalties will be recomputed on next plan)", ransac_radius_);
    } else if (parameter.get_name() == name_ + ".centroid_k") {
      centroid_k_ = parameter.as_int();
      penalties_dirty_ = true;
      RCLCPP_INFO(node_->get_logger(), "Updated centroid_k to %d (penalties will be recomputed on next plan)", centroid_k_);
    } else if (parameter.get_name() == name_ + ".centroid_lambda") {
      centroid_lambda_ = parameter.as_double();
      penalties_dirty_ = true;
      RCLCPP_INFO(node_->get_logger(), "Updated centroid_lambda to %.2f (penalties will be recomputed on next plan)", centroid_lambda_);
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
