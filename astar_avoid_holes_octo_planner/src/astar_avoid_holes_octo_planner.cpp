#include "astar_avoid_holes_octo_planner.h"
#include <pluginlib/class_list_macros.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "GetPose.h"
#include <queue>
#include <cmath>
#include <octomap_msgs/msg/octomap.hpp>

float octile(mbf_octo_core::GraphNode* n, mbf_octo_core::GraphNode* goal) {
    float dx = abs(n->center.x() - goal->center.x());
    float dy = abs(n->center.y() - goal->center.y());
    float diag = (dx < dy) ? dx : dy;
    return (dx + dy) + (sqrt(2) - 2) * diag;
    //      Manhattan    Diagonal-Korrektur
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
    // const octomap::OcTree* tree = mapper_->getOctree();
    // double voxel_size = mapper_->getVoxelSize();
    std::string frame = mapper_->getMapFrame();
    std::array<double,3> min_b, max_b;
    mapper_->getBounds(min_b,max_b);

    for (const auto & [id, node] : graph->nodes) {
        if (!node.is_walkable) continue;

        // octomap::point3d pos = node.center;  // 3D Position in Metern
        // double penalty = graph->node_penalty.at(id);  // Kostenwert für A*
    }
    double start_x = start.pose.position.x;
    double goal_x = goal.pose.position.x;
    
    start_x = std::max(start_x, min_b[0]);
    start_x = std::min(start_x, max_b[0]);
    goal_x = std::max(goal_x, min_b[0]);
    goal_x = std::min(goal_x, max_b[0]);
    tolerance = 1.0;
    cost = 0.0;
    tolerance = 0.0;
    plan.clear();
    message = "Plan created successfully";

    double dist = 0.0; 
    GetPose get_pose_node;
    std::priority_queue<std::pair<std::string,double>, std::vector<std::pair<std::string,double>>, std::greater<std::pair<std::string,double>>> q;
    std::set<std::string> expanded;

    auto transform = get_pose_node.get_pose("base_link", "front left touch sensor");
    auto node =  findNearestNode(graph, octomap::point3d(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z));
    auto goal_node =  findNearestNode(graph, octomap::point3d(goal.pose.position.x, goal.pose.position.y, graph->nodes[node].center.z()));
    graph->nodes[node].is_walkable = true;
    double planning_height = graph->nodes[node].center.z(); 

    reduceTo2DGraph(graph, planning_height);
    floodFill(graph, min_b, max_b);
    inflateLevel(graph);
    bool success = false;
    while(!q.empty()){
        auto [id, cost] = q.top();
        q.pop();

        if (expanded.count(id)) continue;
        expanded.insert(id);

        if (id == goal_node){
            success = true;
            break;
        }

        for (const auto & neighbor_id : graph->adj[id]){
            if (expanded.count(neighbor_id) || graph->nodes[neighbor_id].center.z() != planning_height) continue;  
            double new_cost = cost + graph->node_penalty[neighbor_id] + octile(&graph->nodes[neighbor_id], &graph->nodes[node]);
            q.push({neighbor_id, new_cost});
        }
        if (cancel_planning_){
            return 50;
        }
    }
    if (!success){
        cancel_planning_ = true;
    }

    if (cancel_planning_){
        return 50;
    }
    return 0;
}


void inflateLevel(std::shared_ptr<mbf_octo_core::GraphData>& graph){
    std::set<std::string> to_inflate;
    std::set<std::string> inflated;
    int step = 1;
    for (const auto & [id, node] : graph->nodes){
        if (!node.is_walkable) {
            for (const auto & neighbor_id : graph->adj[id]){
                if (graph->nodes[neighbor_id].is_walkable){
                    to_inflate.insert(neighbor_id); 
                }
            }
        }
    }

    while(!to_inflate.empty()){
        for (const auto & id : to_inflate){
            graph->node_penalty[id] = 10000.0 * 1/pow(2, step); // Exponential penalty
            for (const auto & neighbor_id : graph->adj[id]){
                if(!inflated.count(neighbor_id) && graph->nodes[neighbor_id].is_walkable){
                    to_inflate.insert(neighbor_id);
                } 
            }
            to_inflate.erase(id);
            inflated.insert(id);
        }
         
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
    return true;
}

void AstarAvoidHolesOctoPlanner::flattenPath(){
    const octomap::OcTree* original = mapper_->getOctree();
    octomap::OcTree local_tree(*original);
    auto graph = mapper_->getReadyGraph();
    // octomap::KeyRay ray;
    // octomap::point3d origin, endpoint;
    // origin = octomap::point3d(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z); // Startpunkt
    // std::vector<octomap::point3d> endpoints; // Pfadpunkte


    // for (const auto & key : ray) {
    //     auto node = local_tree.search(key);
        
    //     if (!node) {
    //         // Voxel existiert nicht → Lücke → auffüllen
    //         local_tree.updateNode(key, true);  // als belegt markieren
    //     }
    // }
}

void AstarAvoidHolesOctoPlanner::createNewNode(const std::shared_ptr<mbf_octo_core::GraphData>& graph, double x, double y, double z){
    mbf_octo_core::GraphNode new_node;
    new_node.center = octomap::point3d(x,y,z);
    new_node.is_walkable = true;
    new_node.is_stair_step = false;
    new_node.size = mapper_->getVoxelSize();
    
    std::string new_id = new_node.id();
    graph->nodes[new_id] = new_node;
    graph->node_penalty[new_id] = 0.0;



    //graph->adj[new_id].push_back(existing_node_id);
    //graph->adj[existing_none_id].push_back(new_id);
}

std::string AstarAvoidHolesOctoPlanner::findNearestNode(const std::shared_ptr<mbf_octo_core::GraphData>& graph, const octomap::point3d& query){
    std::string best_id;
    double best_dist = std::numeric_limits<double>::max();
    
    for (const auto& [id, node] : graph->nodes){
        if (!node.is_walkable) continue;

        double dx = node.center.x() - query.x();
        double dy = node.center.y() - query.y();
        double dz = node.center.z() - query.z();

        double dist = dx*dx + dy*dy + dz*dz;

        if (dist < best_dist){
            best_dist = dist;
            best_id = id;
        }
    }
    return best_id;
}

void AstarAvoidHolesOctoPlanner::deleteGraphNodeByID(const std::shared_ptr<mbf_octo_core::GraphData>& graph, std::string id){  
    graph->nodes.erase(id);
    graph->adj.erase(id);
    graph->node_penalty.erase(id);
     
    for (auto & [id, neighbors] : graph->adj){
        neighbors.erase(
            std::remove(neighbors.begin(),neighbors.end(), id),
            neighbors.end()
        );
    }
}

void AstarAvoidHolesOctoPlanner::deleteGraphNodeByPose(const std::shared_ptr<mbf_octo_core::GraphData>& graph, const octomap::point3d query){
    std::string node_id = findNearestNode(graph, query);
    deleteGraphNodeByID(graph, node_id);
}

void AstarAvoidHolesOctoPlanner::reduceTo2DGraph(std::shared_ptr<mbf_octo_core::GraphData>& graph, double z){
    for (const auto & [id, node] : graph->nodes){
        if (node.center.z() != z){
            deleteGraphNodeByID(graph, id);
        }
    }
}

void AstarAvoidHolesOctoPlanner::floodFill(std::shared_ptr<mbf_octo_core::GraphData>& graph, std::array<double,3> min_bound, std::array<double,3> max_bound){
    std::string min_id = findNearestNode(graph, octomap::point3d(min_bound[0], min_bound[1], min_bound[2])); 
    std::string max_id = findNearestNode(graph, octomap::point3d(max_bound[0], max_bound[1], max_bound[2])); 
    std::queue<std::string> q;
    std::set<std::string> visited;

    q.push(min_id);
    visited.insert(min_id);

    while(!q.empty()){
        std::string id = q.front();
        q.pop();

        for (const auto & neighbor_id : graph->adj[id]){
            if (visited.count(neighbor_id)) continue;  
            visited.insert(neighbor_id);
            if (!graph->nodes[neighbor_id].is_walkable) q.push(neighbor_id);
        }
    }

    for (const auto & [id, node] : graph->nodes){
        if (!visited.count(id)){
            graph->nodes[id].is_walkable = true;
        }
    } 

} // namespace astar_planner

PLUGINLIB_EXPORT_CLASS(astar_planner::AstarAvoidHolesOctoPlanner, mbf_octo_core::OctoPlanner)