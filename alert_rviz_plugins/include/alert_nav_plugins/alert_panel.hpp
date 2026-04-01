#pragma once

#include <rviz_common/panel.hpp>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QTimer>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mbf_msgs/action/get_path.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <bring_up_alert_nav/srv/start_nav.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace alert_nav_plugins
{

class AlertPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit AlertPanel(QWidget* parent = nullptr);
  virtual ~AlertPanel();

  // load and save config
  virtual void load(const rviz_common::Config& config) override;
  virtual void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onToggleOctomap();
  void onFactorChanged(double v);
  void onRadiusChanged(double v);
  void onPlanToFrame();
  void onExecPath();
  void onCancelPath();
  void updateButtonUI(bool enabled);

private:
  QPushButton* toggle_btn_ = nullptr;
  QPushButton* plan_btn_ = nullptr;
  QPushButton* exec_path_btn_ = nullptr;
  QPushButton* cancel_path_btn_ = nullptr;
  QDoubleSpinBox* factor_spin_ = nullptr;
  QDoubleSpinBox* radius_spin_ = nullptr;
  QLineEdit* frame_input_ = nullptr;

  // rclcpp node and parameter client
  // rclcpp node and parameter client
  rclcpp::Node::SharedPtr rcl_node_;
  std::shared_ptr<rclcpp::AsyncParametersClient> param_client_;
  // planner node (kept for other parameter operations), and remote node to control
  std::string planner_node_name_ = "octo_planner";
  // node and parameter key used for toggling octomap updates in the robot stack
  std::string param_node_name_ = "/move_base_flex";
  std::string param_key_ = "octo_planner.enable_octomap_updates";
  bool octomap_enabled_ = true;
  QTimer* spin_timer_ = nullptr;
  // true if this panel called rclcpp::init()
  bool we_initialized_rclcpp_ = false;

  using GetPath = mbf_msgs::action::GetPath;
  rclcpp_action::Client<GetPath>::SharedPtr get_path_client_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string map_frame_ = "map";
  std::string get_path_action_name_ = "/move_base_flex/get_path";

  using StartNav = bring_up_alert_nav::srv::StartNav;
  rclcpp::Client<StartNav>::SharedPtr exec_path_client_;
  std::string exec_path_service_name_ = "/exe_path/start_nav";

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cancel_path_client_;
  std::string cancel_path_service_name_ = "/exe_path/cancel_nav";
};

} // namespace alert_nav_plugins
