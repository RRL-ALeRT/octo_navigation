/*
 *  Copyright 2025, Mascor Institute
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
 *  author: Mascor Institute
 */

#include <vector_field_mesh_planner/vector_field_mesh_planner.h>

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <mbf_msgs/action/get_path.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

PLUGINLIB_EXPORT_CLASS(vector_field_mesh_planner::VectorFieldMeshPlanner, mbf_octo_core::MeshPlanner);

namespace vector_field_mesh_planner
{

VectorFieldMeshPlanner::VectorFieldMeshPlanner()
{
}

VectorFieldMeshPlanner::~VectorFieldMeshPlanner()
{
}

bool VectorFieldMeshPlanner::initialize(const std::string& plugin_name,
                                        const rclcpp::Node::SharedPtr& node,
                                        const mbf_octo_core::MeshMapper::Ptr& mapper)
{
  name_   = plugin_name;
  node_   = node;
  mapper_ = mapper;

  cost_weight_ = node_->declare_parameter(name_ + ".cost_weight", cost_weight_);
  lethal_cost_threshold_ = node_->declare_parameter(
    name_ + ".lethal_cost_threshold", lethal_cost_threshold_);
  max_vertex_search_dist_ = node_->declare_parameter(
    name_ + ".max_vertex_search_dist", max_vertex_search_dist_);
  enable_footprint_check_ = node_->declare_parameter(
    name_ + ".enable_footprint_check", enable_footprint_check_);
  robot_height_ = node_->declare_parameter(name_ + ".robot_height", robot_height_);
  robot_width_ = node_->declare_parameter(name_ + ".robot_width", robot_width_);
  collision_margin_ = node_->declare_parameter(name_ + ".collision_margin", collision_margin_);
  headroom_margin_ = node_->declare_parameter(name_ + ".headroom_margin", headroom_margin_);

  path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    "~/plan", rclcpp::QoS(1).transient_local());

  RCLCPP_INFO(node_->get_logger(),
    "VectorFieldMeshPlanner initialized (name=%s, cost_weight=%.2f, "
    "lethal_cost_threshold=%.2f, max_vertex_search_dist=%.2f, "
    "enable_footprint_check=%s, robot_height=%.2f, robot_width=%.2f, collision_margin=%.2f, "
    "headroom_margin=%.2f)",
    name_.c_str(), cost_weight_, lethal_cost_threshold_, max_vertex_search_dist_,
    enable_footprint_check_ ? "true" : "false", robot_height_, robot_width_, collision_margin_,
    headroom_margin_);
  return true;
}

bool VectorFieldMeshPlanner::cancel()
{
  cancel_planning_ = true;
  return true;
}

bool VectorFieldMeshPlanner::findNearestVertex(const mbf_octo_core::MeshGraphData& graph,
                                               const geometry_msgs::msg::Point& p,
                                               uint32_t& out_idx) const
{
  const auto& verts = graph.mesh.mesh_geometry.vertices;
  double best_dist_sq = max_vertex_search_dist_ * max_vertex_search_dist_;
  bool found = false;
  for (size_t i = 0; i < verts.size(); ++i) {
    double dx = verts[i].x - p.x;
    double dy = verts[i].y - p.y;
    double dz = verts[i].z - p.z;
    double d2 = dx * dx + dy * dy + dz * dz;
    if (d2 <= best_dist_sq) {
      best_dist_sq = d2;
      out_idx = static_cast<uint32_t>(i);
      found = true;
    }
  }
  return found;
}

std::vector<geometry_msgs::msg::PoseStamped> VectorFieldMeshPlanner::chainToPoses(
  const mbf_octo_core::MeshGraphData& graph,
  const std::vector<uint32_t>& chain,
  const std::string& frame_id,
  const rclcpp::Time& stamp) const
{
  const auto& verts = graph.mesh.mesh_geometry.vertices;
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  poses.reserve(chain.size());
  for (size_t i = 0; i < chain.size(); ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = frame_id;
    ps.header.stamp = stamp;
    ps.pose.position.x = verts[chain[i]].x;
    ps.pose.position.y = verts[chain[i]].y;
    ps.pose.position.z = verts[chain[i]].z;

    // Orientation from the segment direction (look-ahead, or look-back for
    // the final pose).
    double yaw = 0.0;
    if (i + 1 < chain.size()) {
      double dx = verts[chain[i + 1]].x - verts[chain[i]].x;
      double dy = verts[chain[i + 1]].y - verts[chain[i]].y;
      yaw = std::atan2(dy, dx);
    } else if (i > 0) {
      double dx = verts[chain[i]].x - verts[chain[i - 1]].x;
      double dy = verts[chain[i]].y - verts[chain[i - 1]].y;
      yaw = std::atan2(dy, dx);
    }
    ps.pose.orientation.z = std::sin(yaw * 0.5);
    ps.pose.orientation.w = std::cos(yaw * 0.5);

    poses.push_back(ps);
  }
  return poses;
}

uint32_t VectorFieldMeshPlanner::makePlan(const geometry_msgs::msg::PoseStamped& start,
                                          const geometry_msgs::msg::PoseStamped& goal,
                                          double /*tolerance*/,
                                          std::vector<geometry_msgs::msg::PoseStamped>& plan,
                                          double& cost,
                                          std::string& message)
{
  cancel_planning_ = false;
  plan.clear();
  cost = 0.0;

  if (!mapper_) {
    message = "Mapper not set";
    RCLCPP_WARN(node_->get_logger(), "%s", message.c_str());
    return mbf_msgs::action::GetPath::Result::FAILURE;
  }

  auto graph = mapper_->getReadyMeshGraph();
  if (!graph || graph->vertexCount() == 0) {
    message = "Mesh graph not ready yet";
    RCLCPP_WARN(node_->get_logger(), "%s", message.c_str());
    return mbf_msgs::action::GetPath::Result::NO_PATH_FOUND;
  }

  uint32_t start_idx = 0, goal_idx = 0;
  if (!findNearestVertex(*graph, start.pose.position, start_idx)) {
    message = "No mesh vertex within max_vertex_search_dist of start";
    RCLCPP_WARN(node_->get_logger(), "%s", message.c_str());
    return mbf_msgs::action::GetPath::Result::INVALID_START;
  }
  if (!findNearestVertex(*graph, goal.pose.position, goal_idx)) {
    message = "No mesh vertex within max_vertex_search_dist of goal";
    RCLCPP_WARN(node_->get_logger(), "%s", message.c_str());
    return mbf_msgs::action::GetPath::Result::INVALID_GOAL;
  }

  const auto& vertex_costs = graph->vertex_costs.mesh_vertex_costs.costs;
  const size_t n = graph->vertexCount();
  if (vertex_costs.size() != n) {
    message = "Vertex cost layer does not match mesh vertex count";
    RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
    return mbf_msgs::action::GetPath::Result::INTERNAL_ERROR;
  }
  if (graph->clearance_above.size() != n || graph->clearance_lateral.size() != n) {
    message = "Clearance layer does not match mesh vertex count";
    RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
    return mbf_msgs::action::GetPath::Result::INTERNAL_ERROR;
  }

  // Robot footprint: a vertex the body physically can't occupy is a hard
  // exclusion, not something cost_weight_ merely discourages. Skipped
  // entirely when enable_footprint_check_ is false.
  const float min_lateral_clear = static_cast<float>(robot_width_ / 2.0 + collision_margin_);
  auto fitsFootprint = [&](uint32_t idx) {
    if (!enable_footprint_check_) return true;
    return graph->clearance_above[idx] >= static_cast<float>(robot_height_) &&
           graph->clearance_lateral[idx] >= min_lateral_clear;
  };

  // Headroom as a soft cost, independent of enable_footprint_check_: a
  // vertex can pass the hard clearance check (plenty tall enough) while
  // still being somewhere the robot shouldn't be encouraged to go — squeezed
  // right under a railing's top rail is the textbook case. Ramps from 0 at
  // robot_height_ + headroom_margin_ (or more) of clearance up to 1.0 (as
  // costly as the worst wall/corner) right at the bare minimum.
  auto headroomCost = [&](uint32_t idx) -> float {
    if (headroom_margin_ <= 0.0) return 0.0f;
    double excess = graph->clearance_above[idx] - robot_height_;
    if (excess >= headroom_margin_) return 0.0f;
    if (excess <= 0.0) return 1.0f;
    return static_cast<float>(1.0 - excess / headroom_margin_);
  };

  if (!fitsFootprint(start_idx)) {
    message = "Start position does not have enough clearance for the robot footprint";
    RCLCPP_WARN(node_->get_logger(),
      "%s (clearance_above=%.2f, clearance_lateral=%.2f, need height>=%.2f, lateral>=%.2f)",
      message.c_str(), graph->clearance_above[start_idx], graph->clearance_lateral[start_idx],
      robot_height_, min_lateral_clear);
    return mbf_msgs::action::GetPath::Result::BLOCKED_START;
  }
  if (!fitsFootprint(goal_idx)) {
    message = "Goal position does not have enough clearance for the robot footprint";
    RCLCPP_WARN(node_->get_logger(),
      "%s (clearance_above=%.2f, clearance_lateral=%.2f, need height>=%.2f, lateral>=%.2f)",
      message.c_str(), graph->clearance_above[goal_idx], graph->clearance_lateral[goal_idx],
      robot_height_, min_lateral_clear);
    return mbf_msgs::action::GetPath::Result::BLOCKED_GOAL;
  }

  // ---- Dijkstra on Gv, early-exit once goal_idx is finalized -----------------
  std::vector<double> dist(n, std::numeric_limits<double>::infinity());
  std::vector<int64_t> parent(n, -1);
  std::vector<bool> visited(n, false);

  struct QueueEntry { double dist; uint32_t idx; };
  auto cmp = [](const QueueEntry& a, const QueueEntry& b) { return a.dist > b.dist; };
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, decltype(cmp)> pq(cmp);

  dist[start_idx] = 0.0;
  pq.push({0.0, start_idx});

  bool reached_goal = (start_idx == goal_idx);
  while (!reached_goal && !pq.empty()) {
    if (cancel_planning_) {
      message = "Planning canceled";
      return mbf_msgs::action::GetPath::Result::CANCELED;
    }
    QueueEntry top = pq.top();
    pq.pop();
    if (visited[top.idx]) continue;
    visited[top.idx] = true;
    if (top.idx == goal_idx) {
      reached_goal = true;
      break;
    }

    float top_headroom_cost = headroomCost(top.idx);

    for (const auto& edge : graph->vertex_adjacency[top.idx]) {
      if (visited[edge.to]) continue;
      if (!fitsFootprint(edge.to)) continue;  // robot's body doesn't fit here
      float vcost = std::max({vertex_costs[top.idx], vertex_costs[edge.to],
                               top_headroom_cost, headroomCost(edge.to)});
      if (vcost >= static_cast<float>(lethal_cost_threshold_)) continue;  // impassable
      double w = static_cast<double>(edge.length) * (1.0 + cost_weight_ * vcost);
      double nd = dist[top.idx] + w;
      if (nd < dist[edge.to]) {
        dist[edge.to] = nd;
        parent[edge.to] = static_cast<int64_t>(top.idx);
        pq.push({nd, edge.to});
      }
    }
  }

  const std::string map_frame = mapper_->getMapFrame();
  const std::string path_frame = map_frame.empty() ? start.header.frame_id : map_frame;
  const rclcpp::Time now = node_->now();
  const auto& verts = graph->mesh.mesh_geometry.vertices;

  if (!reached_goal) {
    size_t reachable = static_cast<size_t>(std::count(visited.begin(), visited.end(), true));
    message = "No path found on mesh graph";
    RCLCPP_WARN(node_->get_logger(),
      "%s (start_idx=%u cost=%.2f, goal_idx=%u cost=%.2f, reached %zu/%zu vertices, "
      "lethal_cost_threshold=%.2f) — likely blocked by the lethal cutoff rather than "
      "a genuinely disconnected mesh; try raising lethal_cost_threshold if a route "
      "should exist through a costly area (e.g. a doorway/corner).",
      message.c_str(), start_idx, vertex_costs[start_idx], goal_idx, vertex_costs[goal_idx],
      reachable, n, lethal_cost_threshold_);

    // Publish a partial path to whichever reached vertex is geometrically
    // closest to the goal, so ~/plan shows in RViz roughly where the search
    // got stuck instead of just vanishing.
    uint32_t closest_idx = start_idx;
    double closest_dist_sq = std::numeric_limits<double>::max();
    for (size_t i = 0; i < n; ++i) {
      if (!visited[i]) continue;
      double dx = verts[i].x - goal.pose.position.x;
      double dy = verts[i].y - goal.pose.position.y;
      double dz = verts[i].z - goal.pose.position.z;
      double d2 = dx * dx + dy * dy + dz * dz;
      if (d2 < closest_dist_sq) { closest_dist_sq = d2; closest_idx = static_cast<uint32_t>(i); }
    }

    std::vector<uint32_t> partial_chain;
    for (int64_t v = static_cast<int64_t>(closest_idx); v != -1; v = parent[v]) {
      partial_chain.push_back(static_cast<uint32_t>(v));
      if (static_cast<uint32_t>(v) == start_idx) break;
    }
    std::reverse(partial_chain.begin(), partial_chain.end());

    nav_msgs::msg::Path partial_path;
    partial_path.header.frame_id = path_frame;
    partial_path.header.stamp = now;
    partial_path.poses = chainToPoses(*graph, partial_chain, path_frame, now);
    path_pub_->publish(partial_path);

    RCLCPP_WARN(node_->get_logger(),
      "Published partial path (%zu poses, closest_idx=%u dist_to_goal=%.2f) on ~/plan "
      "— that's roughly where the search got stuck.",
      partial_path.poses.size(), closest_idx, std::sqrt(closest_dist_sq));

    return mbf_msgs::action::GetPath::Result::NO_PATH_FOUND;
  }

  // ---- Backtrack parent chain -------------------------------------------------
  std::vector<uint32_t> chain;
  for (int64_t v = static_cast<int64_t>(goal_idx); v != -1; v = parent[v]) {
    chain.push_back(static_cast<uint32_t>(v));
    if (static_cast<uint32_t>(v) == start_idx) break;
  }
  std::reverse(chain.begin(), chain.end());

  plan = chainToPoses(*graph, chain, path_frame, now);
  cost = dist[goal_idx];

  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = path_frame;
  path_msg.header.stamp = now;
  path_msg.poses = plan;
  path_pub_->publish(path_msg);

  RCLCPP_INFO(node_->get_logger(),
    "VectorFieldMeshPlanner: plan found (%zu poses, cost=%.3f)", plan.size(), cost);

  return mbf_msgs::action::GetPath::Result::SUCCESS;
}

}  // namespace vector_field_mesh_planner
