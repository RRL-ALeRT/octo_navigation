#include "avoid_holes_exploration_planner.h"
#include <pluginlib/class_list_macros.hpp>
#include <octomap/OcTree.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <queue>
#include <unordered_set>
#include <algorithm>
#include <cmath>

namespace astar_planner {

bool AvoidHolesExplorationPlanner::initialize(
    const std::string & name,
    const rclcpp::Node::SharedPtr & node,
    const mbf_octo_core::OctoMapper::Ptr & mapper)
{
    // Base sets up name_/node_/mapper_ and the graph/penalty/path publishers.
    AstarAvoidHolesOctoPlanner::initialize(name, node, mapper);
    frontier_marker_pub_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>("frontier_markers", 1);
    return true;
}

uint32_t AvoidHolesExplorationPlanner::makePlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    double tolerance,
    std::vector<geometry_msgs::msg::PoseStamped> & plan,
    double & cost,
    std::string & message)
{
    // This is an explorer: the caller-supplied goal is IGNORED. The plugin picks
    // its own goal (a frontier, pulled back to a safe node) and plans to it.
    (void)goal;
    (void)tolerance;

    auto graph = mapper_->getReadyGraph();
    cost = 0.0;
    plan.clear();
    cancel_planning_ = false;

    // 1. Snap the robot onto the graph the same way the path finder does: prefer the
    //    next walkable node IN FRONT of Spot. A plain 3D-nearest lookup from base_link
    //    (~0.5 m above the floor) can grab an elevated node beside the robot instead of
    //    the floor, which drags robot_z — and with it every hole-fill node — too high.
    std::string start_node = findNodeInFront(graph, start);
    if (start_node.empty()) {
        // Fall back to nearest node WITHIN the feet-height band (see startZBand): never
        // snap onto a platform Spot is not standing on when his floor is unscanned.
        const auto [z_min, z_max] = startZBand(start);
        start_node = findNearestNode(graph,
            octomap::point3d(start.pose.position.x,
                             start.pose.position.y,
                             start.pose.position.z),
            z_min, z_max);
    }
    if (start_node.empty()) {
        message = "No walkable start node at standing height - floor not scanned yet?";
        return 50;
    }
    graph->nodes[start_node].is_walkable = true;
    double robot_z = graph->nodes[start_node].center.z();

    // Latch the FIRST request's height as the exploration floor, and keep it as the
    // lowest ever seen. The graph is pruned relative to THIS floor - not the robot's
    // current height - so once Spot climbs onto a higher platform the lower map is NOT
    // pruned away and it can still come back down (otherwise reduceTo2DGraph would kill
    // most of the map and strand it up top).
    if (!has_floor_z_) { exploration_floor_z_ = robot_z; has_floor_z_ = true; }
    else               { exploration_floor_z_ = std::min(exploration_floor_z_, robot_z); }

    // 2. Reform the graph: keep everything from the exploration floor upward, zero
    //    penalties, fill holes on the robot's current plane, inflate borders.
    prepareGraph(graph, exploration_floor_z_, robot_z);

    // 3. Pick the frontier node from the reformed graph + the live octree.
    const octomap::OcTree* octree = mapper_->getOctree();
    if (!octree) {
        message = "No octree available for frontier detection";
        return 50;
    }

    // 3. Collect ALL frontier candidates (fills last_frontier_candidates_).
    selectFrontierNode(graph, octree);
    if (last_frontier_candidates_.empty()) {
        message = "No frontier found - map may be fully explored";
        return 50;
    }

    // Rank candidates in three hard tiers, then by score within each tier:
    //   tier 0: within max_frontier_dist_ AND inside the front cone -> always tried first
    //   tier 1: within max_frontier_dist_ but off to the side/behind
    //   tier 2: everything else (far away)
    // So Spot only plans ahead of himself (or slightly to the side) as long as any such
    // frontier exists; side/behind and far frontiers are pure fallback so exploration
    // doesn't dead-end when the space in front is exhausted.
    const double rx = start.pose.position.x;
    const double ry = start.pose.position.y;
    const auto& q  = start.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double hx = std::cos(yaw), hy = std::sin(yaw);   // robot heading (unit)
    const double cone_min_align = std::cos(front_cone_half_deg_ * M_PI / 180.0);

    double max_unknown = 1.0;
    for (const auto& [id, u] : last_frontier_candidates_)
        max_unknown = std::max(max_unknown, static_cast<double>(u));

    // Radius expansion: inside each tier candidates are grouped into distance rings
    // (frontier_ring_step_ wide). The nearest non-empty ring wins, so the search starts
    // at the lowest possible radius and opens up ring by ring toward max_frontier_dist_;
    // score only orders candidates WITHIN one ring.
    struct Ranked { int tier; int ring; double score; std::string id; };
    std::vector<Ranked> scored;
    scored.reserve(last_frontier_candidates_.size());
    int tier0 = 0, tier1 = 0;
    for (const auto& [id, u] : last_frontier_candidates_) {
        auto it = graph->nodes.find(id);
        if (it == graph->nodes.end()) continue;
        double vx = it->second.center.x() - rx;
        double vy = it->second.center.y() - ry;
        double n  = std::hypot(vx, vy);
        double alignment = (n > 1e-6) ? (hx * vx + hy * vy) / n : 0.0;   // [-1,1], ahead=+1
        double dir_term  = 0.5 * (alignment + 1.0);                       // [0,1]
        double unk_term  = static_cast<double>(u) / max_unknown;          // [0,1]
        double score = heading_weight_ * dir_term + (1.0 - heading_weight_) * unk_term;

        const bool near     = n <= max_frontier_dist_;
        const bool in_front = alignment >= cone_min_align;
        const int  tier     = (near && in_front) ? 0 : (near ? 1 : 2);
        const int  ring     = static_cast<int>(n / frontier_ring_step_);
        if (tier == 0) ++tier0; else if (tier == 1) ++tier1;
        scored.push_back({tier, ring, score, id});
    }
    std::sort(scored.begin(), scored.end(),
              [](const Ranked& a, const Ranked& b){
                  if (a.tier != b.tier) return a.tier < b.tier;
                  if (a.ring != b.ring) return a.ring < b.ring;
                  return a.score > b.score;
              });

    RCLCPP_INFO(node_->get_logger(),
        "Explore ranking: %d front(<=%.1fm, +-%.0fdeg), %d near-side, %zu far/behind",
        tier0, max_frontier_dist_, front_cone_half_deg_,
        tier1, scored.size() - tier0 - tier1);

    // 4. Walk the candidates. For each, pull the frontier back to a low-penalty node
    //    and test whether A* can actually REACH it from the robot. Take the first that
    //    works. Candidates too close to one already tried are skipped, so we sample
    //    DISTINCT frontier regions instead of many adjacent nodes (and cap attempts).
    const double cluster_dist2 = frontier_cluster_dist_ * frontier_cluster_dist_;
    std::vector<octomap::point3d> tried_centers;
    std::string chosen_frontier, chosen_goal;
    int attempts = 0;

    for (const auto& ranked : scored) {
        const std::string& fid = ranked.id;
        auto fit = graph->nodes.find(fid);
        if (fit == graph->nodes.end()) continue;
        const octomap::point3d fc = fit->second.center;

        bool clustered = false;
        for (const auto& c : tried_centers) {
            double dx = fc.x() - c.x(), dy = fc.y() - c.y(), dz = fc.z() - c.z();
            if (dx*dx + dy*dy + dz*dz < cluster_dist2) { clustered = true; break; }
        }
        if (clustered) continue;
        tried_centers.push_back(fc);

        if (++attempts > max_frontier_attempts_) break;

        std::string goal_node = nearestLowPenaltyNode(graph, fid);
        if (goal_node.empty()) continue;
        // Skip frontiers whose pulled-back goal is basically where the robot already is:
        // driving there is no real progress and the loop would replan it forever. Forces
        // the planner to pick a farther, genuinely new frontier instead.
        if ((graph->nodes[goal_node].center - graph->nodes[start_node].center).norm()
                < min_travel_dist_)
            continue;

        std::string msg;
        double c = 0.0;
        std::vector<geometry_msgs::msg::PoseStamped> candidate_plan;
        if (runAstar(graph, start_node, goal_node, candidate_plan, c, msg) == 0) {
            plan = std::move(candidate_plan);   // runAstar already published this path
            cost = c;
            chosen_frontier = fid;
            chosen_goal = goal_node;
            break;
        }
    }

    if (chosen_frontier.empty()) {
        message = "No reachable frontier (tried " + std::to_string(attempts) +
                  " of " + std::to_string(last_frontier_candidates_.size()) + " candidates)";
        return 50;
    }

    RCLCPP_INFO(node_->get_logger(),
        "Explore: chose frontier %s -> goal %s after %d attempt(s) (%zu candidates)",
        chosen_frontier.c_str(), chosen_goal.c_str(), attempts,
        last_frontier_candidates_.size());

    // 5. Visualize the frontier field, the chosen frontier and the pulled-back goal.
    publishFrontierMarkers(graph, chosen_frontier, chosen_goal);
    message = "Frontier plan created";
    return 0;
}

std::string AvoidHolesExplorationPlanner::selectFrontierNode(
    const std::shared_ptr<mbf_octo_core::GraphData>& graph,
    const octomap::OcTree* octree)
{
    const double vs = mapper_->getVoxelSize();

    last_frontier_candidates_.clear();

    std::string best_id;
    // require strictly more than (min_unknown_ - 1) so a node needs >= min_unknown_.
    int best_unknown = min_unknown_ - 1;

    for (const auto& [id, node] : graph->nodes) {
        if (!node.is_walkable) continue;

        // Count unknown (== never-observed) cells in the horizontal ring around
        // the node at its own level. A cell is unknown when the octree has no
        // node there (search() == nullptr) - neither free nor occupied.
        int unknown = 0;
        for (int dx = -unknown_radius_; dx <= unknown_radius_; ++dx) {
            for (int dy = -unknown_radius_; dy <= unknown_radius_; ++dy) {
                if (dx == 0 && dy == 0) continue;
                octomap::point3d p(node.center.x() + dx * vs,
                                   node.center.y() + dy * vs,
                                   node.center.z());
                if (octree->search(p) == nullptr) ++unknown;
            }
        }

        if (unknown >= min_unknown_)
            last_frontier_candidates_.push_back({id, unknown});

        if (unknown > best_unknown) {
            best_unknown = unknown;
            best_id = id;
        }
    }

    if (!best_id.empty())
        RCLCPP_INFO(node_->get_logger(),
            "selectFrontierNode: best node %s with %d unknown cells (radius=%d)",
            best_id.c_str(), best_unknown, unknown_radius_);

    return best_id;
}

std::string AvoidHolesExplorationPlanner::nearestLowPenaltyNode(
    const std::shared_ptr<mbf_octo_core::GraphData>& graph,
    const std::string& from)
{
    if (graph->nodes.find(from) == graph->nodes.end()) return "";

    auto penalty_of = [&](const std::string& id) {
        auto it = graph->node_penalty.find(id);
        return it != graph->node_penalty.end() ? it->second : 0.0;
    };

    std::queue<std::string> q;
    std::unordered_set<std::string> visited;
    q.push(from);
    visited.insert(from);

    // Fallback: the lowest-penalty node seen, used ONLY if nothing under the cap is
    // reachable ("take a more-penalty node only if it really needs to").
    std::string best_id = from;
    double best_penalty = penalty_of(from);

    // BFS outward. Penalty falls off inward from the edge, so the FIRST node we reach
    // whose penalty is at/below the cap is the closest safe node - pull back exactly as
    // far as needed to stand on a low-penalty (safe) node, no further.
    while (!q.empty()) {
        std::string id = q.front();
        q.pop();

        double p = penalty_of(id);
        if (p <= acceptable_penalty_) {
            RCLCPP_INFO(node_->get_logger(),
                "nearestLowPenaltyNode: safe node %s (penalty %.1f <= cap %.1f)",
                id.c_str(), p, acceptable_penalty_);
            return id;
        }
        if (p < best_penalty) { best_penalty = p; best_id = id; }

        for (const auto& neighbor_id : graph->adj[id]) {
            if (visited.count(neighbor_id)) continue;
            auto it = graph->nodes.find(neighbor_id);
            if (it == graph->nodes.end() || !it->second.is_walkable) continue;
            visited.insert(neighbor_id);
            q.push(neighbor_id);
        }
    }

    // No node under the cap is reachable from this frontier: settle for the least-bad.
    RCLCPP_INFO(node_->get_logger(),
        "nearestLowPenaltyNode: no node under cap %.1f; using min-penalty %s (%.1f)",
        acceptable_penalty_, best_id.c_str(), best_penalty);
    return best_id;
}

void AvoidHolesExplorationPlanner::publishFrontierMarkers(
    const std::shared_ptr<mbf_octo_core::GraphData>& graph,
    const std::string& frontier_node,
    const std::string& goal_node)
{
    if (!frontier_marker_pub_) return;

    const std::string frame = mapper_->getMapFrame();
    const auto now = node_->get_clock()->now();
    const double vs = mapper_->getVoxelSize();

    visualization_msgs::msg::MarkerArray array;

    // Clear the previous batch so stale candidates don't linger.
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = frame;
    clear.header.stamp = now;
    clear.ns = "frontier";
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    // Highest unknown count in this batch, for colour scaling.
    int max_unknown = 1;
    for (const auto& [id, u] : last_frontier_candidates_)
        max_unknown = std::max(max_unknown, u);

    // --- All candidate frontiers as a single SPHERE_LIST (brighter = more unknown) ---
    visualization_msgs::msg::Marker candidates;
    candidates.header.frame_id = frame;
    candidates.header.stamp = now;
    candidates.ns = "frontier";
    candidates.id = 0;
    candidates.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    candidates.action = visualization_msgs::msg::Marker::ADD;
    candidates.scale.x = candidates.scale.y = candidates.scale.z = vs * 0.6;
    candidates.pose.orientation.w = 1.0;
    for (const auto& [id, u] : last_frontier_candidates_) {
        auto it = graph->nodes.find(id);
        if (it == graph->nodes.end()) continue;
        geometry_msgs::msg::Point p;
        p.x = it->second.center.x();
        p.y = it->second.center.y();
        p.z = it->second.center.z();
        candidates.points.push_back(p);
        double t = static_cast<double>(u) / max_unknown;   // 0..1
        std_msgs::msg::ColorRGBA c;
        c.r = static_cast<float>(t);          // more unknown -> yellow
        c.g = static_cast<float>(t);
        c.b = static_cast<float>(1.0 - t);    // less unknown -> blue
        c.a = 0.9f;
        candidates.colors.push_back(c);
    }
    array.markers.push_back(candidates);

    auto sphere_at = [&](const std::string& id, int marker_id, double s,
                         float r, float g, float b) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame;
        m.header.stamp = now;
        m.ns = "frontier";
        m.id = marker_id;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        auto it = graph->nodes.find(id);
        if (it != graph->nodes.end()) {
            m.pose.position.x = it->second.center.x();
            m.pose.position.y = it->second.center.y();
            m.pose.position.z = it->second.center.z();
        }
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = s;
        m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0f;
        array.markers.push_back(m);
    };

    // Selected frontier (magenta) and pulled-back goal (green).
    sphere_at(frontier_node, 1, vs * 2.0, 1.0f, 0.0f, 1.0f);
    sphere_at(goal_node,     2, vs * 2.0, 0.0f, 1.0f, 0.0f);

    // Arrow from frontier -> goal so the "pull back" is obvious.
    auto fit = graph->nodes.find(frontier_node);
    auto git = graph->nodes.find(goal_node);
    if (fit != graph->nodes.end() && git != graph->nodes.end()) {
        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = frame;
        arrow.header.stamp = now;
        arrow.ns = "frontier";
        arrow.id = 3;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.scale.x = vs * 0.4;   // shaft diameter
        arrow.scale.y = vs * 0.8;   // head diameter
        arrow.scale.z = vs * 0.8;   // head length
        arrow.color.r = 1.0f; arrow.color.g = 1.0f; arrow.color.b = 1.0f; arrow.color.a = 1.0f;
        geometry_msgs::msg::Point p0, p1;
        p0.x = fit->second.center.x(); p0.y = fit->second.center.y(); p0.z = fit->second.center.z();
        p1.x = git->second.center.x(); p1.y = git->second.center.y(); p1.z = git->second.center.z();
        arrow.points.push_back(p0);
        arrow.points.push_back(p1);
        array.markers.push_back(arrow);
    }

    frontier_marker_pub_->publish(array);
    RCLCPP_INFO(node_->get_logger(),
        "publishFrontierMarkers: %zu candidate frontiers",
        last_frontier_candidates_.size());
}

} // namespace astar_planner

PLUGINLIB_EXPORT_CLASS(astar_planner::AvoidHolesExplorationPlanner, mbf_octo_core::OctoPlanner)
