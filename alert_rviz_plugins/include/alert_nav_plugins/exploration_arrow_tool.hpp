#pragma once

#include <rviz_default_plugins/rviz_default_plugins/tools/pose/pose_tool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace alert_nav_plugins
{

class ExplorationArrowTool : public rviz_default_plugins::tools::PoseTool
{
  Q_OBJECT

public:
  ExplorationArrowTool();
  ~ExplorationArrowTool() override;

  void onInitialize() override;

private:
  void onPoseSet(double x, double y, double theta) override;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

}  // namespace alert_nav_plugins
