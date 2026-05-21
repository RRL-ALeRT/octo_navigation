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

private:
    std::string name_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    mbf_octo_core::OctoMapper::Ptr mapper_;
    std::atomic_bool cancel_planning_{false};
    std::string findNearestNode(const std::shared_ptr<mbf_octo_core::GraphData>& graph, const octomap::point3d& query);
    void createNewNode(const std::shared_ptr<mbf_octo_core::GraphData>& graph, double x, double y, double z);
    void deleteGraphNodeByID(const std::shared_ptr<mbf_octo_core::GraphData>& graph, std::string id);
    void deleteGraphNodeByPose(const std::shared_ptr<mbf_octo_core::GraphData>& graph, const octomap::point3d query);
    void reduceTo2DGraph(std::shared_ptr<mbf_octo_core::GraphData>& graph, double z);
    void floodFill(std::shared_ptr<mbf_octo_core::GraphData>& graph, std::array<double,3> min_bound, std::array<double,3> max_bound);
    void flattenPath();
};

} // namespace astar_planner

#endif