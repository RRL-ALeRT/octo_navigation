#include "alert_nav_plugins/movebase_goal_tool.hpp"

#include <rviz_common/display_context.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace alert_nav_plugins
{

MoveBaseGoalTool::MoveBaseGoalTool()
{
  shortcut_key_ = 'm';
}

void MoveBaseGoalTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("MoveBase Goal");
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/movebase_goal", 10);
}

void MoveBaseGoalTool::onPoseSet(double x, double y, double theta)
{
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = context_->getFixedFrame().toStdString();
  goal.header.stamp = node_->now();
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  goal.pose.orientation = tf2::toMsg(q);
  pub_->publish(goal);
  RCLCPP_INFO(node_->get_logger(),
    "MoveBase Goal arrow set: (%.2f, %.2f, yaw %.2f) -> /movebase_goal", x, y, theta);
}

}  // namespace alert_nav_plugins

PLUGINLIB_EXPORT_CLASS(alert_nav_plugins::MoveBaseGoalTool, rviz_common::Tool)
