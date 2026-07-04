#pragma once

#include <rviz_default_plugins/tools/pose/pose_tool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace alert_nav_plugins
{

/**
 * Toolbar arrow tool ("MoveBase Goal"): same drag-an-arrow interaction as
 * 2D Pose Estimate / 2D Goal Pose, but the pose is published on /movebase_goal.
 * exe_path_node picks it up and sends a MoveBase action goal (plan + drive)
 * with the planner currently selected in the alert panel dropdown.
 */
class MoveBaseGoalTool : public rviz_default_plugins::tools::PoseTool
{
public:
  MoveBaseGoalTool();
  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double theta) override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
};

}  // namespace alert_nav_plugins
