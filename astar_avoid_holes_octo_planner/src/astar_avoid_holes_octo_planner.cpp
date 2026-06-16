#include "astar_avoid_holes_octo_planner.h"
#include <pluginlib/class_list_macros.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <queue>
#include <cmath>
#include <limits>
#include <set>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <octomap_msgs/msg/octomap.hpp>

float octile(mbf_octo_core::GraphNode* n, mbf_octo_core::GraphNode* goal) {
    float dx = std::abs(n->center.x() - goal->center.x());
    float dy = std::abs(n->center.y() - goal->center.y());
    float diag = std::min(dx, dy);
    return (dx + dy) + (std::sqrt(2.0f) - 2.0f) * diag;
}

namespace astar_planner {

uint32_t AstarAvoidHolesOctoPlanner::makePlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    double tolerance,
    std::vector<geometry_msgs::msg::PoseStamped> & plan,
    double & cost,
    std::string & message)
{
    auto octree = mapper_->getOctree();
    auto graph = mapper_->getReadyGraph();
    std::string frame = mapper_->getMapFrame();
    std::array<double,3> min_b, max_b;
    mapper_->getBounds(min_b, max_b);

    cost = 0.0;
    plan.clear();
    cancel_planning_ = false;

    std::string start_node = findNearestNode(graph,
        octomap::point3d(start.pose.position.x,
                         start.pose.position.y,
                         start.pose.position.z));
    if (start_node.empty()) {
        message = "No walkable start node found";
        return 50;
    }

    double planning_height = graph->nodes[start_node].center.z();
    graph->nodes[start_node].is_walkable = true;

    std::string goal_node = findNearestNode(graph,
        octomap::point3d(goal.pose.position.x,
                         goal.pose.position.y,
                         planning_height));
    if (goal_node.empty()) {
        message = "No walkable goal node found";
        return 50;
    }
    RCLCPP_INFO(node_->get_logger(), "map wird umgebaut");
    RCLCPP_INFO(node_->get_logger(), "nodes vor reduce: %zu", graph->nodes.size());

    reduceTo2DGraph(graph, planning_height);
    RCLCPP_INFO(node_->get_logger(), "nodes nach reduce: %zu", graph->nodes.size());
    floodFill(graph, min_b, max_b);

    RCLCPP_INFO(node_->get_logger(), "nodes nach floodFill: %zu", graph->nodes.size());
    for (auto& [id, _] : graph->nodes)
        graph->node_penalty[id] = 0.0;
    inflateLevel(graph);
    publishGraphAsOctomap(graph,"map");
    RCLCPP_INFO(node_->get_logger(), "map wurde umgebaut");

    std::priority_queue<std::pair<double,std::string>, std::vector<std::pair<double,std::string>>, std::greater<std::pair<double,std::string>>> open_set; 
    std::unordered_map<std::string, double> g_score;
    std::unordered_map<std::string, std::string> predecessor;
    std::set<std::string> closed;

    g_score[start_node] = 0.0;
    open_set.push({
        octile(&graph->nodes[start_node], &graph->nodes[goal_node]),
        start_node
    });

    bool success = false;

    while (!open_set.empty()) {
        auto [f, id] = open_set.top();
        open_set.pop();

        if (closed.count(id)) continue;
        closed.insert(id);

        if (id == goal_node) { success = true; break; }

        for (const auto & neighbor_id : graph->adj[id]) {
            if (!graph->nodes[neighbor_id].is_walkable) continue;
            if (closed.count(neighbor_id)) continue;

            double edge_cost = graph->node_penalty[neighbor_id]
                + octile(&graph->nodes[neighbor_id], &graph->nodes[id]);
            double tentative_g = g_score[id] + edge_cost;

            if (!g_score.count(neighbor_id) || tentative_g < g_score[neighbor_id]) {
                g_score[neighbor_id] = tentative_g;
                predecessor[neighbor_id] = id;
                double h = octile(&graph->nodes[neighbor_id], &graph->nodes[goal_node]);
                open_set.push({tentative_g + h, neighbor_id});
            }
        }
        //if (cancel_planning_) return 50;
    }

    if (!success) { message = "No path found"; return 50; }

    std::vector<std::string> path_nodes;
    std::string current = goal_node;
    while (current != start_node) {
        path_nodes.push_back(current);
        if (!predecessor.count(current)) { message = "Broken path reconstruction"; return 50; }
        current = predecessor[current];
    }
    path_nodes.push_back(start_node);
    std::reverse(path_nodes.begin(), path_nodes.end());

    nav_msgs::msg::Path path;
    for (const auto & pid : path_nodes) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame;
        pose.header.stamp = node_->get_clock()->now();
        pose.pose.position.x = graph->nodes[pid].center.x();
        pose.pose.position.y = graph->nodes[pid].center.y();
        pose.pose.position.z = graph->nodes[pid].center.z();
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
        path.poses.push_back(pose);
    }
    path_pub_->publish(path);
    cost = g_score[goal_node];
    std::stringstream ss;
    ss << cost;
    const char* str = ss.str().c_str();
    RCLCPP_INFO(node_->get_logger(), str);
    publishPenaltyMap(graph, "map");
    message = "A* plan created successfully";
    return 0;
}



void AstarAvoidHolesOctoPlanner::publishGraphAsOctomap(
    const std::shared_ptr<mbf_octo_core::GraphData>& graph,
    const std::string& frame)
{
    double voxel_size = mapper_->getVoxelSize();
    RCLCPP_INFO(node_->get_logger(), "voxel_size: %f", mapper_->getVoxelSize());

// Ersten paar Node-Koordinaten ausgeben:
    int count = 0;
    for (const auto& [id, node] : graph->nodes) {
        if (count++ > 5) break;
        RCLCPP_INFO(node_->get_logger(), "node: x=%f y=%f z=%f walkable=%d",
            node.center.x(), node.center.y(), node.center.z(), node.is_walkable);
    }
    octomap::OcTree tree(voxel_size);

    for (const auto& [id, node] : graph->nodes) {
        if (node.is_walkable)
            tree.updateNode(node.center, true);
    }
    tree.updateInnerOccupancy();

    octomap_msgs::msg::Octomap msg;
    msg.header.frame_id = frame;
    msg.header.stamp = node_->get_clock()->now();

    RCLCPP_INFO(node_->get_logger(), "Octomap tree size: %zu", tree.size());
    RCLCPP_INFO(node_->get_logger(), "walkable nodes: %zu", graph->nodes.size());
    if (!octomap_msgs::binaryMapToMsg(tree, msg)) {
        RCLCPP_ERROR(node_->get_logger(), "binaryMapToMsg fehlgeschlagen!");
        return;
    }
    RCLCPP_INFO(node_->get_logger(), "Octomap published, size: %zu", tree.size());
    octomap_pub_->publish(msg);
}

void AstarAvoidHolesOctoPlanner::inflateLevel(
    std::shared_ptr<mbf_octo_core::GraphData>& graph)
{
    std::set<std::string> frontier;
    std::set<std::string> inflated;
    int step = 1;

    // Rand-Nodes: walkable, aber nicht alle 8 Nachbarn walkable
    for (const auto& [id, node] : graph->nodes) {
        if (!node.is_walkable) continue;
        
        int walkable_neighbors = 0;
        for (const auto& nid : graph->adj[id])
            if (graph->nodes[nid].is_walkable)
                walkable_neighbors++;
        
        if (walkable_neighbors < 8)  // oder < graph->adj[id].size()
            frontier.insert(id);
    }

    RCLCPP_INFO(node_->get_logger(), "frontier size: %zu", frontier.size());

    while (!frontier.empty()) {
        std::set<std::string> next;
        for (const auto& id : frontier) {
            graph->node_penalty[id] = 10000.0 / std::pow(2.0, step);
            for (const auto& nid : graph->adj[id])
                if (graph->nodes[nid].is_walkable && !inflated.count(nid))
                    next.insert(nid);
            inflated.insert(id);
        }
        frontier = next;
        step++;
    }
}

bool AstarAvoidHolesOctoPlanner::cancel()
{
    cancel_planning_ = true;
    return true;
}

bool AstarAvoidHolesOctoPlanner::initialize(
    const std::string & name,
    const rclcpp::Node::SharedPtr & node,
    const mbf_octo_core::OctoMapper::Ptr & mapper)
{
    name_ = name;
    node_ = node;
    mapper_ = mapper;
    path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("plan", 1);
    octomap_pub_ = node_->create_publisher<octomap_msgs::msg::Octomap>("graph_octomap", 1);
    penalty_pub_ = node_->create_publisher<octomap_msgs::msg::Octomap>("penalty_octomap", 1);
    return true;
}

void AstarAvoidHolesOctoPlanner::publishPenaltyMap(
    const std::shared_ptr<mbf_octo_core::GraphData>& graph,
    const std::string& frame)
{
    double voxel_size = mapper_->getVoxelSize();
    octomap::ColorOcTree tree(voxel_size);

    for (const auto& [id, node] : graph->nodes) {
        if (!node.is_walkable) continue;
        auto* n = tree.updateNode(node.center, true);
        
        double penalty = graph->node_penalty.count(id) ? graph->node_penalty.at(id) : 0.0;
        // Penalty auf Farbe mappen: 0 = grün, hoch = rot
        uint8_t r = std::min(255.0, penalty * 5.0);
        uint8_t g = std::max(0.0, 255.0 - penalty * 5.0);
        n->setColor(r, g, 0);
    }
    tree.updateInnerOccupancy();

    octomap_msgs::msg::Octomap msg;
    msg.header.frame_id = frame;
    msg.header.stamp = node_->get_clock()->now();
    octomap_msgs::fullMapToMsg(tree, msg);  // full statt binary für Farben
    penalty_pub_->publish(msg);
}

void AstarAvoidHolesOctoPlanner::createNewNode(
    const std::shared_ptr<mbf_octo_core::GraphData>& graph,
    double x, double y, double z)
{
    mbf_octo_core::GraphNode new_node;
    new_node.center = octomap::point3d(x, y, z);
    new_node.is_walkable = true;
    new_node.is_stair_step = false;
    new_node.size = mapper_->getVoxelSize();
    std::string new_id = new_node.id();
    graph->nodes[new_id] = new_node;
    graph->node_penalty[new_id] = 0.0;
}

std::string AstarAvoidHolesOctoPlanner::findNearestNode(
    const std::shared_ptr<mbf_octo_core::GraphData>& graph,
    const octomap::point3d& query)
{
    std::string best_id;
    double best_dist = std::numeric_limits<double>::max();
    for (const auto& [id, node] : graph->nodes) {
        if (!node.is_walkable) continue;
        double dx = node.center.x() - query.x();
        double dy = node.center.y() - query.y();
        double dz = node.center.z() - query.z();
        double d = dx*dx + dy*dy + dz*dz;
        if (d < best_dist) { best_dist = d; best_id = id; }
    }
    return best_id;
}

void AstarAvoidHolesOctoPlanner::deleteGraphNodeByID(
    const std::shared_ptr<mbf_octo_core::GraphData>& graph,
    const std::string node_id)
{
    graph->nodes.erase(node_id);
    graph->node_penalty.erase(node_id);
    for (auto & [adj_id, neighbors] : graph->adj)
        neighbors.erase(std::remove(neighbors.begin(), neighbors.end(), node_id), neighbors.end());
    graph->adj.erase(node_id);
}

void AstarAvoidHolesOctoPlanner::deleteGraphNodeByPose(
    const std::shared_ptr<mbf_octo_core::GraphData>& graph,
    const octomap::point3d query)
{
    std::string nid = findNearestNode(graph, query);
    if (!nid.empty()) deleteGraphNodeByID(graph, nid);
}

void AstarAvoidHolesOctoPlanner::reduceTo2DGraph(
    std::shared_ptr<mbf_octo_core::GraphData>& graph, double z)
{
    std::unordered_set<std::string> keep;
    for (const auto& [id, node] : graph->nodes)
        if (std::abs(node.center.z() - z) <= 0.001)
            keep.insert(id);

    for (auto it = graph->nodes.begin(); it != graph->nodes.end();)
        it = keep.count(it->first) ? std::next(it) : graph->nodes.erase(it);

    for (auto it = graph->node_penalty.begin(); it != graph->node_penalty.end();)
        it = keep.count(it->first) ? std::next(it) : graph->node_penalty.erase(it);

    for (auto it = graph->adj.begin(); it != graph->adj.end();) {
        if (!keep.count(it->first)) { it = graph->adj.erase(it); continue; }
        auto& nb = it->second;
        nb.erase(std::remove_if(nb.begin(), nb.end(),
            [&](const std::string& n){ return !keep.count(n); }), nb.end());
        ++it;
    }
}

void AstarAvoidHolesOctoPlanner::floodFill(
    std::shared_ptr<mbf_octo_core::GraphData>& graph,
    std::array<double,3> min_bound,
    std::array<double,3> max_bound)
{
    // Start von einem walkable Randknoten
    std::string start_id;
    double best_dist = std::numeric_limits<double>::max();
    octomap::point3d center(
        (min_bound[0] + max_bound[0]) / 2.0,
        (min_bound[1] + max_bound[1]) / 2.0,
        min_bound[2]);

    for (const auto& [id, node] : graph->nodes) {
        if (!node.is_walkable) continue;
        double dx = node.center.x() - center.x();
        double dy = node.center.y() - center.y();
        double d = dx*dx + dy*dy;
        if (d < best_dist) { best_dist = d; start_id = id; }
    }

    if (start_id.empty()) return;

    // Flood von walkable start node aus
    std::queue<std::string> q;
    std::set<std::string> visited;
    q.push(start_id);
    visited.insert(start_id);

    while (!q.empty()) {
        std::string id = q.front(); q.pop();
        for (const auto & neighbor_id : graph->adj[id]) {
            if (visited.count(neighbor_id)) continue;
            visited.insert(neighbor_id);
            if (graph->nodes[neighbor_id].is_walkable)
                q.push(neighbor_id);
        }
    }

    // Nicht erreichbare walkable Nodes = Löcher → unwalkable
    for (auto & [id, node] : graph->nodes)
        if (node.is_walkable && !visited.count(id))
            node.is_walkable = false;
}

} // namespace astar_planner

PLUGINLIB_EXPORT_CLASS(astar_planner::AstarAvoidHolesOctoPlanner, mbf_octo_core::OctoPlanner)