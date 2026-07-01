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
    auto graph = mapper_->getReadyGraph();

    cost = 0.0;
    plan.clear();
    cancel_planning_ = false;

    // Snap the start to the next walkable node IN FRONT of Spot (along his heading),
    // not merely the nearest one: the ground directly beneath him may be a hole with no
    // node at all, so we begin the plan from a node ahead of him.
    std::string start_node = findNodeInFront(graph, start);
    if (start_node.empty()) {
        // Nothing ahead in the forward cone (e.g. facing a wall/edge) — fall back to
        // the plain nearest walkable node so planning can still proceed.
        start_node = findNearestNode(graph,
            octomap::point3d(start.pose.position.x,
                             start.pose.position.y,
                             start.pose.position.z));
    }
    if (start_node.empty()) {
        message = "No walkable start node found";
        return 50;
    }

    double planning_height = graph->nodes[start_node].center.z();
    graph->nodes[start_node].is_walkable = true;

    // Resolve the goal at its REAL height so a goal on a different level snaps to
    // that level instead of collapsing onto the start's plane.
    std::string goal_node = findNearestNode(graph,
        octomap::point3d(goal.pose.position.x,
                         goal.pose.position.y,
                         goal.pose.position.z));
    if (goal_node.empty()) {
        message = "No walkable goal node found";
        return 50;
    }
    graph->nodes[goal_node].is_walkable = true;

    // Keep every surface from the LOWER of {start level, goal level} upward, so both
    // endpoints and everything connecting them survive; prune only what is below.
    double goal_height = graph->nodes[goal_node].center.z();
    double reduce_height = std::min(planning_height, goal_height);
    prepareGraph(graph, reduce_height, planning_height);
    return runAstar(graph, start_node, goal_node, plan, cost, message);
}

void AstarAvoidHolesOctoPlanner::prepareGraph(
    std::shared_ptr<mbf_octo_core::GraphData>& graph,
    double reduce_height, double planning_height)
{
    reduceTo2DGraph(graph, reduce_height);
    for (auto& [id, _] : graph->nodes)
        graph->node_penalty[id] = 0.0;
    publishGraphAsOctomap(graph, "map");
    publishPenaltyMap(graph, "map", pre_fill_penalty_pub_);
    fillSingleHoles(graph, planning_height);
    inflateLevel(graph);
}

uint32_t AstarAvoidHolesOctoPlanner::runAstar(
    std::shared_ptr<mbf_octo_core::GraphData>& graph,
    const std::string& start_node, const std::string& goal_node,
    std::vector<geometry_msgs::msg::PoseStamped>& plan,
    double& cost, std::string& message)
{
    std::string frame = mapper_->getMapFrame();

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
    path.header.stamp = node_->get_clock()->now();
    path.header.frame_id = "map";
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
    RCLCPP_INFO(node_->get_logger(), "A* plan cost: %f", cost);
    publishPenaltyMap(graph, "map", penalty_pub_);
    message = "A* plan created successfully";
    return 0;
}



void AstarAvoidHolesOctoPlanner::publishGraphAsOctomap(
    const std::shared_ptr<mbf_octo_core::GraphData>& graph,
    const std::string& frame)
{
    double voxel_size = mapper_->getVoxelSize();
    RCLCPP_INFO(node_->get_logger(), "voxel_size: %f", mapper_->getVoxelSize());

    octomap::OcTree tree(voxel_size);

    for (const auto& [id, node] : graph->nodes) {
        if (node.is_walkable)
            tree.updateNode(node.center, true);
    }
    tree.updateInnerOccupancy();

    octomap_msgs::msg::Octomap msg;
    msg.header.frame_id = frame;
    msg.header.stamp = node_->get_clock()->now();

    if (!octomap_msgs::binaryMapToMsg(tree, msg)) {
        RCLCPP_ERROR(node_->get_logger(), "binaryMapToMsg fehlgeschlagen!");
        return;
    }
    RCLCPP_INFO(node_->get_logger(), "Octomap published, size: %zu", tree.size());
    octomap_pub_->publish(msg);
}

void AstarAvoidHolesOctoPlanner::fillSingleHoles(
    std::shared_ptr<mbf_octo_core::GraphData>& graph,
    double planning_height)
{
    double vs = mapper_->getVoxelSize();
    double tol = vs * 0.6;

    // Alle vorhandenen Positionen indexieren
    std::unordered_map<std::string, octomap::point3d> positions;
    for (const auto& [id, node] : graph->nodes)
        if (node.is_walkable)
            positions[id] = node.center;

    // 8 Nachbar-Offsets
    std::vector<std::pair<double,double>> offsets = {
        {-vs,-vs},{0,-vs},{vs,-vs},
        {-vs, 0},         {vs, 0},
        {-vs, vs},{0, vs},{vs, vs}
    };

    std::vector<octomap::point3d> to_create;

    for (const auto& [id, pos] : positions) {
        if (graph->adj[id].size() < 7) continue;  // zu viele fehlen

        for (const auto& [ox, oy] : offsets) {
            double tx = pos.x() + ox;
            double ty = pos.y() + oy;

            // Gibt es bereits einen Node an dieser Position?
            bool found = false;
            for (const auto& nid : graph->adj[id]) {
                auto& np = graph->nodes[nid].center;
                if (std::abs(np.x() - tx) < tol && std::abs(np.y() - ty) < tol) {
                    found = true; break;
                }
            }
            if (!found)
                to_create.push_back({tx, ty, planning_height});
        }
    }
    for (const auto& p : to_create) {
        std::string new_id = createNewNode(graph, p.x(), p.y(), p.z());
        graph->adj[new_id];   // ensure the new node's adjacency entry exists

        // Wire adjacency BOTH ways with every surrounding walkable node in the 8-ring
        // (within ~1.5 voxels in x and y). `positions` already contains earlier-created
        // hole nodes, so adjacent fills connect to each other too.
        for (const auto& [existing_id, existing_pos] : positions) {
            if (existing_id == new_id) continue;
            double dx = std::abs(p.x() - existing_pos.x());
            double dy = std::abs(p.y() - existing_pos.y());
            if (dx > vs + tol || dy > vs + tol) continue;   // not a neighbour

            auto& new_adj = graph->adj[new_id];
            auto& ex_adj  = graph->adj[existing_id];        // ensures entry exists
            if (std::find(new_adj.begin(), new_adj.end(), existing_id) == new_adj.end())
                new_adj.push_back(existing_id);             // new -> surrounding
            if (std::find(ex_adj.begin(), ex_adj.end(), new_id) == ex_adj.end())
                ex_adj.push_back(new_id);                   // surrounding -> new
        }
        positions[new_id] = p;
    }
}

void AstarAvoidHolesOctoPlanner::inflateLevel(
    std::shared_ptr<mbf_octo_core::GraphData>& graph)
{
    double voxel_size = mapper_->getVoxelSize();
    // Inflation follows the connected WALKABLE SURFACE, not a single flat plane.
    // A node's neighbourhood is the immediate 8-ring — horizontal distance within
    // ~1.5 voxels — allowing up to TWO steps (max_dz) of vertical change. This keeps
    // floor, ramps and stair treads (and a step up to a level ~2 voxels higher) as one
    // continuous manifold, so the middle-of-corridor ridge carries smoothly across the
    // step instead of the step-up nodes looking like borders and inflating (which spikes
    // the cost and pushes A* off the real path). A drop/rise larger than this is still a
    // REAL border. We still exclude voxels stacked (near-)directly above/below (small
    // horizontal offset) so a platform sitting above the floor doesn't merge with it.
    // NOTE: the graph's own adjacency is far denser than a grid (step edges out to
    // ~3 cells), so restricting to this immediate ring is essential — otherwise even
    // edge nodes keep ≥8 neighbours and nothing is ever flagged as a border.
    const double min_horiz_sq = std::pow(voxel_size * 0.5, 2.0);
    const double max_horiz_sq = std::pow(voxel_size * 1.5, 2.0);
    const double max_dz       = voxel_size * 2.5;   // up to ~two levels of vertical change

    auto is_surface_neighbor = [&](const mbf_octo_core::GraphNode& a,
                                   const mbf_octo_core::GraphNode& b) {
        if (!b.is_walkable) return false;
        if (std::abs(b.center.z() - a.center.z()) > max_dz) return false;
        double dx = b.center.x() - a.center.x();
        double dy = b.center.y() - a.center.y();
        double dxy2 = dx * dx + dy * dy;
        return dxy2 >= min_horiz_sq && dxy2 <= max_horiz_sq;
    };

    auto walkable_plane_neighbors = [&](const mbf_octo_core::GraphNode& node,
                                        const std::string& id) {
        int count = 0;
        for (const auto& nid : graph->adj[id]) {
            auto it = graph->nodes.find(nid);
            if (it == graph->nodes.end()) continue;
            if (is_surface_neighbor(node, it->second)) ++count;
        }
        return count;
    };

    std::set<std::string> frontier;
    std::set<std::string> inflated;
    int step = 1;

    // Border = fewer than border_min_neighbors walkable surface-neighbours in the immediate
    // ring. Using 7 (not 8) means a node missing a single neighbour (a lone hole beside it)
    // is NOT treated as a border, so inflation only fires on real edges. Lower to 6 to be
    // even more tolerant of holes next to the path.
    const int border_min_neighbors = 7;
    for (const auto& [id, node] : graph->nodes) {
        if (!node.is_walkable) continue;
        if (walkable_plane_neighbors(node, id) < border_min_neighbors)
            frontier.insert(id);
    }

    RCLCPP_INFO(node_->get_logger(), "frontier size: %zu", frontier.size());

    while (!frontier.empty()) {
        std::set<std::string> next;
        for (const auto& fid : frontier) {
            graph->node_penalty[fid] = 30000.0 / std::pow(2.0, step);
            auto fit = graph->nodes.find(fid);
            if (fit != graph->nodes.end()) {
                for (const auto& nid : graph->adj[fid]) {
                    auto it = graph->nodes.find(nid);
                    if (it == graph->nodes.end()) continue;
                    if (!is_surface_neighbor(fit->second, it->second)) continue;
                    if (!inflated.count(nid))
                        next.insert(nid);
                }
            }
            inflated.insert(fid);
        }
        frontier = next;
        step++;
    }

    // --- Diagnostics: understand the graph's vertical structure & spread -----
    size_t walkable = 0, penalized = 0;
    double max_penalty = 0.0;
    std::map<int, int> z_hist;                 // walkable node count per z-layer (mm)
    std::map<int, int> neighbor_hist;          // #nodes by same-plane walkable-neighbor count
    for (const auto& [id, node] : graph->nodes) {
        if (!node.is_walkable) continue;
        ++walkable;
        double p = graph->node_penalty.count(id) ? graph->node_penalty.at(id) : 0.0;
        if (p > 0.0) { ++penalized; max_penalty = std::max(max_penalty, p); }
        z_hist[static_cast<int>(std::lround(node.center.z() * 1000.0))]++;
        int n = walkable_plane_neighbors(node, id);
        neighbor_hist[std::min(n, 9)]++;       // bucket 9 = "9 or more"
    }
    std::stringstream zs;
    for (const auto& [zmm, c] : z_hist) zs << " z=" << zmm << "mm:" << c;
    std::stringstream ns;
    for (const auto& [n, c] : neighbor_hist) ns << " " << n << ":" << c;
    RCLCPP_INFO(node_->get_logger(),
        "inflateLevel diag: walkable=%zu penalized(>0)=%zu inflated=%zu max_penalty=%.1f steps=%d",
        walkable, penalized, inflated.size(), max_penalty, step);
    RCLCPP_INFO(node_->get_logger(), "inflateLevel diag: z-layers (mm:count):%s", zs.str().c_str());
    RCLCPP_INFO(node_->get_logger(), "inflateLevel diag: same-plane walkable-neighbor histogram (count:nodes):%s", ns.str().c_str());
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
    path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("~/path", 1);
    octomap_pub_ = node_->create_publisher<octomap_msgs::msg::Octomap>("graph_octomap", 1);
    penalty_pub_ = node_->create_publisher<octomap_msgs::msg::Octomap>("penalty_octomap", 1);
    pre_fill_penalty_pub_ = node_->create_publisher<octomap_msgs::msg::Octomap>("pre_fill_penalty_octomap", 1);
    return true;
}

void AstarAvoidHolesOctoPlanner::publishPenaltyMap(
    const std::shared_ptr<mbf_octo_core::GraphData>& graph,
    const std::string& frame, const std::shared_ptr<rclcpp::Publisher<octomap_msgs::msg::Octomap>>& pub)
{
    double voxel_size = mapper_->getVoxelSize();
    octomap::ColorOcTree tree(voxel_size);

    for (const auto& [id, node] : graph->nodes) {
        if (!node.is_walkable) continue;
        auto* n = tree.updateNode(node.center, true);
        
        double penalty = graph->node_penalty.count(id) ? graph->node_penalty.at(id) : 0.0;
        // Penalty auf Farbe mappen: 0 = grün, hoch = rot

        uint8_t r = static_cast<uint8_t>(std::min(255.0, penalty / 5.0));
        uint8_t g = static_cast<uint8_t>(std::max(0.0, 255.0 - penalty / 5.0));


        n->setColor(r, g, 0);
    }
    tree.updateInnerOccupancy();

    octomap_msgs::msg::Octomap msg;
    msg.header.frame_id = frame;
    msg.header.stamp = node_->get_clock()->now();
    octomap_msgs::fullMapToMsg(tree, msg);  // full statt binary für Farben
    pub->publish(msg);
}

std::string AstarAvoidHolesOctoPlanner::createNewNode(
    const std::shared_ptr<mbf_octo_core::GraphData>& graph,
    double x, double y, double z)
{
    mbf_octo_core::GraphNode new_node;
    new_node.center = octomap::point3d(x, y, z);
    new_node.is_walkable = true;
    new_node.is_stair_step = false;
    new_node.size = mapper_->getVoxelSize();

    char idbuf[128];
    int ix_mm = static_cast<int>(std::round(x * 1000.0));
    int iy_mm = static_cast<int>(std::round(y * 1000.0));
    int iz_mm = static_cast<int>(std::round(z * 1000.0));
    std::snprintf(idbuf, sizeof(idbuf), "c_%d_%d_%d", ix_mm, iy_mm, iz_mm);
    std::string new_id(idbuf);

    graph->nodes[new_id] = new_node;
    graph->node_penalty[new_id] = 0.0;
    return new_id;
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

std::string AstarAvoidHolesOctoPlanner::findNodeInFront(
    const std::shared_ptr<mbf_octo_core::GraphData>& graph,
    const geometry_msgs::msg::PoseStamped& start)
{
    // Robot forward direction (yaw) extracted from the start orientation quaternion.
    const auto& q = start.pose.orientation;
    double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    double fx = std::cos(yaw), fy = std::sin(yaw);

    const double sx = start.pose.position.x;
    const double sy = start.pose.position.y;
    const double sz = start.pose.position.z;

    // Accept only nodes inside a forward cone (half-angle ~50 deg) so we snap to a node
    // AHEAD of Spot, never behind or directly beside him.
    const double min_cos = std::cos(50.0 * M_PI / 180.0);
    // Ignore nodes essentially under the robot; we want the *next* node forward.
    const double min_forward = mapper_->getVoxelSize() * 0.5;

    std::string best_id;
    double best_dist = std::numeric_limits<double>::max();
    for (const auto& [id, node] : graph->nodes) {
        if (!node.is_walkable) continue;
        double dx = node.center.x() - sx;
        double dy = node.center.y() - sy;
        double dxy = std::sqrt(dx * dx + dy * dy);
        if (dxy < min_forward) continue;                 // too close / under Spot
        if ((dx * fx + dy * fy) / dxy < min_cos) continue;  // outside the forward cone
        double dz = node.center.z() - sz;
        double d = dx * dx + dy * dy + dz * dz;
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

void AstarAvoidHolesOctoPlanner::floodFill(std::shared_ptr<mbf_octo_core::GraphData>& graph){
    if (graph->nodes.empty()) return;

    const double vs = mapper_->getVoxelSize();
    if (vs <= 0.0) return;

    // Close holes with a per-level morphological CLOSING: grow the floor into empty cells
    // (dilate) closing_radius times, then shrink it back (erode) the same amount. Dilation
    // fills corner cells first and then expands inward, so it closes holes of ANY size up to
    // ~2*closing_radius wide — not just fully-surrounded single cells. Erosion removes the
    // growth that spilled past the real boundary, so the platform does NOT expand outward.
    // New cells become cheap (penalty 0) walkable nodes at the surrounding height.
    //
    // Everything is PER Z-LEVEL (the cell key carries the z-level): a hole is only closed
    // from floor on its OWN platform, and the node sits at that platform's height — otherwise
    // a different-height surface would be counted and a node dropped in mid-air.
    const int closing_radius = 3;   // fills holes up to ~2*this cells wide (corners included)

    // Cell key packs (ix, iy, level) so lookups are same-level only. level = round(z/voxel).
    auto cell_key = [](int ix, int iy, int lvl) -> long long {
        return (static_cast<long long>(ix & 0x1fffff) << 42)
             ^ (static_cast<long long>(iy & 0x1fffff) << 21)
             ^  static_cast<long long>(lvl & 0x1fffff);
    };
    auto idx = [&](double v){ return static_cast<int>(std::lround(v / vs)); };
    const int nbr8[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};

    struct Cell { int ix, iy, lvl; };
    std::unordered_set<long long> floor_cells;
    std::unordered_map<long long, Cell> cell_pos;                        // key -> (ix,iy,lvl)
    std::unordered_map<long long, double> cell_z;                        // representative z per cell
    std::unordered_map<long long, std::vector<std::string>> floor_nodes; // original nodes per cell
    std::unordered_map<long long, std::string> new_node_id;             // node created per filled cell
    int min_ix = std::numeric_limits<int>::max(), max_ix = std::numeric_limits<int>::lowest();
    int min_iy = std::numeric_limits<int>::max(), max_iy = std::numeric_limits<int>::lowest();
    int min_lvl = std::numeric_limits<int>::max(), max_lvl = std::numeric_limits<int>::lowest();
    for (const auto& [id, node] : graph->nodes) {
        if (!node.is_walkable) continue;
        int ix = idx(node.center.x()), iy = idx(node.center.y());
        int lvl = static_cast<int>(std::lround(node.center.z() / vs));
        long long k = cell_key(ix, iy, lvl);
        floor_cells.insert(k);
        cell_pos[k] = {ix, iy, lvl};
        floor_nodes[k].push_back(id);
        min_ix = std::min(min_ix, ix); max_ix = std::max(max_ix, ix);
        min_iy = std::min(min_iy, iy); max_iy = std::max(max_iy, iy);
        min_lvl = std::min(min_lvl, lvl); max_lvl = std::max(max_lvl, lvl);
    }
    if (floor_cells.empty()) return;
    for (const auto& [k, ids] : floor_nodes) {
        double s = 0.0; for (const auto& id : ids) s += graph->nodes[id].center.z();
        cell_z[k] = s / ids.size();
    }

    (void)min_ix; (void)max_ix; (void)min_iy; (void)max_iy; (void)min_lvl; (void)max_lvl;

    // work = floor + provisional fill; pos/zmap carry position and height for every cell.
    std::unordered_set<long long> work = floor_cells;
    std::unordered_map<long long, Cell> pos = cell_pos;
    std::unordered_map<long long, double> zmap = cell_z;

    // Dilation: grow into empty same-level neighbours, propagating height outward.
    for (int r = 0; r < closing_radius; ++r) {
        std::vector<std::pair<long long, Cell>> add;
        for (long long k : work) {
            const Cell& c = pos[k];
            double z = zmap[k];
            for (const auto& d : nbr8) {
                int nx = c.ix + d[0], ny = c.iy + d[1];
                long long nk = cell_key(nx, ny, c.lvl);
                if (work.count(nk) || zmap.count(nk)) continue;
                zmap[nk] = z;
                add.push_back({nk, {nx, ny, c.lvl}});
            }
        }
        for (auto& [nk, c] : add) { work.insert(nk); pos[nk] = c; }
    }

    // Erosion: remove grown cells exposed to empty space; never touch original floor.
    for (int r = 0; r < closing_radius; ++r) {
        std::vector<long long> rem;
        for (long long k : work) {
            if (floor_cells.count(k)) continue;
            const Cell& c = pos[k];
            for (const auto& d : nbr8) {
                long long nk = cell_key(c.ix + d[0], c.iy + d[1], c.lvl);
                if (!work.count(nk)) { rem.push_back(k); break; }
            }
        }
        for (long long k : rem) work.erase(k);
    }

    // Whatever survived closing that was not original floor is an interior hole → fill it.
    size_t total_created = 0;
    for (long long k : work) {
        if (floor_cells.count(k)) continue;
        const Cell& c = pos[k];
        std::string nid = createNewNode(graph, c.ix * vs, c.iy * vs, zmap[k]);
        cell_pos[k] = c;
        new_node_id[k] = nid;
        graph->adj[nid];
        ++total_created;
    }

    // Wire each filled node to same-level 8-ring: original floor nodes and other filled nodes.
    auto connect = [&](const std::string& a, const std::string& b){
        auto& av = graph->adj[a];
        if (std::find(av.begin(), av.end(), b) == av.end()) av.push_back(b);
    };
    for (const auto& [k, nid] : new_node_id) {
        const Cell& c = cell_pos[k];
        for (const auto& d : nbr8) {
            long long nk = cell_key(c.ix + d[0], c.iy + d[1], c.lvl);
            auto fit = floor_nodes.find(nk);
            if (fit != floor_nodes.end())
                for (const auto& oid : fit->second) { connect(nid, oid); connect(oid, nid); }
            auto nnit = new_node_id.find(nk);
            if (nnit != new_node_id.end() && nnit->second != nid) { connect(nid, nnit->second); connect(nnit->second, nid); }
        }
    }

    RCLCPP_INFO(node_->get_logger(),
        "floodFill: per-level closing(r=%d) filled %zu hole cells",
        closing_radius, total_created);
}

void AstarAvoidHolesOctoPlanner::reduceTo2DGraph(
    std::shared_ptr<mbf_octo_core::GraphData>& graph, double z)
{
    std::unordered_set<std::string> keep;
    for (const auto& [id, node] : graph->nodes)
        if (node.center.z() >= z - 1e-6)   // keep this level and everything above
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


} // namespace astar_planner

PLUGINLIB_EXPORT_CLASS(astar_planner::AstarAvoidHolesOctoPlanner, mbf_octo_core::OctoPlanner)