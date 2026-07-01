#include "alert_nav_plugins/exploration_arrow_tool.hpp"

#include <memory>
#include <string>

#include "rviz_common/display_context.hpp"
#include "rviz_common/load_resource.hpp"

namespace alert_nav_plugins
{

ExplorationArrowTool::ExplorationArrowTool()
: rviz_default_plugins::tools::PoseTool()
{
  shortcut_key_ = 'e';
}

ExplorationArrowTool::~ExplorationArrowTool()
{
}

void ExplorationArrowTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("3D ALeRT Exploration");
  setIcon(rviz_common::loadPixmap("package://rviz_default_plugins/icons/classes/SetGoal.png"));

  // Obtain the background ROS node
  auto rclcpp_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  pose_pub_ = rclcpp_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/alert_exploration_pose", 10);
}

void ExplorationArrowTool::onPoseSet(double x, double y, double theta)
{
  // Create and publish the pose
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = context_->getFixedFrame().toStdString();
  msg.header.stamp = context_->getClock()->now();

  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = 0.0;

  // Convert theta (yaw) to quaternion
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = std::sin(theta / 2.0);
  msg.pose.orientation.w = std::cos(theta / 2.0);

  pose_pub_->publish(msg);
}

}  // namespace alert_nav_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(alert_nav_plugins::ExplorationArrowTool, rviz_common::Tool)
