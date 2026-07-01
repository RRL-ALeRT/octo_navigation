#ifndef OCTO_NAVIGATION__ASTAR_AVOID_HOLES_OCTO_PLANNER_H
#define OCTO_NAVIGATION__ASTAR_AVOID_HOLES_OCTO_PLANNER_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <mbf_octo_core/octo_planner.h>
#include <mbf_octo_core/octo_mapper.h>
#include <mbf_octo_core/octo_graph_data.h>
#include <mbf_msgs/action/get_path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <atomic>
#include <string>
#include <vector>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/conversions.h>

#include <octomap_msgs/conversions.h>

namespace astar_planner {

class AstarAvoidHolesOctoPlanner : public mbf_octo_core::OctoPlanner {
public:
    typedef std::shared_ptr<AstarAvoidHolesOctoPlanner> Ptr;

    bool initialize(const std::string & name,
                    const rclcpp::Node::SharedPtr & node,
                    const mbf_octo_core::OctoMapper::Ptr & mapper) override;

    uint32_t makePlan(const geometry_msgs::msg::PoseStamped & start,
                      const geometry_msgs::msg::PoseStamped & goal,
                      double tolerance,
                      std::vector<geometry_msgs::msg::PoseStamped> & plan,
                      double & cost,
                      std::string & message) override;

    bool cancel() override;

protected:
    std::string name_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
    mbf_octo_core::OctoMapper::Ptr mapper_;
    std::atomic_bool cancel_planning_{false};
    std::string findNearestNode(const std::shared_ptr<mbf_octo_core::GraphData>& graph, const octomap::point3d& query);
    // Snap the start to the nearest walkable node inside a forward cone along the robot's
    // heading (from the start orientation), so navigation begins at a node in front of
    // Spot even when there is no node directly beneath him. Empty if nothing is ahead.
    std::string findNodeInFront(const std::shared_ptr<mbf_octo_core::GraphData>& graph,
                                const geometry_msgs::msg::PoseStamped& start);
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr penalty_pub_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pre_fill_penalty_pub_;
    std::string createNewNode(const std::shared_ptr<mbf_octo_core::GraphData>& graph, double x, double y, double z);
    void deleteGraphNodeByID(const std::shared_ptr<mbf_octo_core::GraphData>& graph, std::string id);
    void deleteGraphNodeByPose(const std::shared_ptr<mbf_octo_core::GraphData>& graph, const octomap::point3d query);
    void reduceTo2DGraph(std::shared_ptr<mbf_octo_core::GraphData>& graph, double z);
    void inflateLevel(std::shared_ptr<mbf_octo_core::GraphData>& graph);
    void floodFill(std::shared_ptr<mbf_octo_core::GraphData>& graph);
    void publishGraphAsOctomap(
        const std::shared_ptr<mbf_octo_core::GraphData>& graph,
        const std::string& frame
    );
    void publishPenaltyMap(
        const std::shared_ptr<mbf_octo_core::GraphData>& graph,
        const std::string& frame,
        const std::shared_ptr<rclcpp::Publisher<octomap_msgs::msg::Octomap>>& pub
    );
    void fillSingleHoles(
        std::shared_ptr<mbf_octo_core::GraphData>& graph,
        double planning_height);

    // Shared graph preparation (prune below reduce_height, zero penalties, fill holes,
    // inflate) and A* search, so subclasses (e.g. the frontier planner) can reuse the
    // exact same "reformed" graph and planner without duplicating makePlan.
    void prepareGraph(std::shared_ptr<mbf_octo_core::GraphData>& graph,
                      double reduce_height, double planning_height);
    uint32_t runAstar(std::shared_ptr<mbf_octo_core::GraphData>& graph,
                      const std::string& start_node, const std::string& goal_node,
                      std::vector<geometry_msgs::msg::PoseStamped>& plan,
                      double& cost, std::string& message);

};

} // namespace astar_planner

#endif