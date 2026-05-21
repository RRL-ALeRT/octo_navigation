#ifndef OCTO_NAVIGATION__ASTAR_AVOID_HOLES_OCTO_PLANNER_H
#define OCTO_NAVIGATION__ASTAR_AVOID_HOLES_OCTO_PLANNER_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <mbf_octo_core/octo_planner.h>
#include <mbf_octo_core/octo_mapper.h>
#include <mbf_octo_core/octo_graph_data.h>
<<<<<<< HEAD
=======
#include <mbf_msgs/action/get_path.hpp>
>>>>>>> 3388536 (clean main push)
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <atomic>
#include <string>
#include <vector>
<<<<<<< HEAD
#include <octomap_msgs/conversions.h>
=======
>>>>>>> 3388536 (clean main push)



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
<<<<<<< HEAD
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
    mbf_octo_core::OctoMapper::Ptr mapper_;
    std::atomic_bool cancel_planning_{false};
    std::string findNearestNode(const std::shared_ptr<mbf_octo_core::GraphData>& graph, const octomap::point3d& query);
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr penalty_pub_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pre_fill_penalty_pub_;
    std::string createNewNode(const std::shared_ptr<mbf_octo_core::GraphData>& graph, double x, double y, double z);
    void deleteGraphNodeByID(const std::shared_ptr<mbf_octo_core::GraphData>& graph, std::string id);
    void deleteGraphNodeByPose(const std::shared_ptr<mbf_octo_core::GraphData>& graph, const octomap::point3d query);
    void reduceTo2DGraph(std::shared_ptr<mbf_octo_core::GraphData>& graph, double z);
    void inflateLevel(std::shared_ptr<mbf_octo_core::GraphData>& graph);
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


=======
    mbf_octo_core::OctoMapper::Ptr mapper_;
    std::atomic_bool cancel_planning_{false};
    std::string findNearestNode(const std::shared_ptr<mbf_octo_core::GraphData>& graph, const octomap::point3d& query);
    void createNewNode(const std::shared_ptr<mbf_octo_core::GraphData>& graph, double x, double y, double z);
    void deleteGraphNodeByID(const std::shared_ptr<mbf_octo_core::GraphData>& graph, std::string id);
    void deleteGraphNodeByPose(const std::shared_ptr<mbf_octo_core::GraphData>& graph, const octomap::point3d query);
    void reduceTo2DGraph(std::shared_ptr<mbf_octo_core::GraphData>& graph, double z);
    void floodFill(std::shared_ptr<mbf_octo_core::GraphData>& graph, std::array<double,3> min_bound, std::array<double,3> max_bound);
    void flattenPath();
>>>>>>> 3388536 (clean main push)
};

} // namespace astar_planner

#endif