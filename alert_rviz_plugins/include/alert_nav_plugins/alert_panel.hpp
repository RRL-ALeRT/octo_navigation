#pragma once

#include <rviz_common/panel.hpp>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QStringList>
#include <QTimer>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mbf_msgs/action/get_path.hpp>
#include <alert_msgs/action/explore_to_goal.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <bring_up_alert_nav/srv/start_nav.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>

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
  void onExplore();
  void onExecPath();
  void onCancelPath();
  void onTraceBack();
  void updateButtonUI(bool enabled);
  void onPlannerChanged(int index);
  void refreshPlannerList();
  void updatePlannerList(QStringList names);

private:
  QPushButton* toggle_btn_ = nullptr;
  QPushButton* plan_btn_ = nullptr;
  QPushButton* explore_btn_ = nullptr;
  QPushButton* exec_path_btn_ = nullptr;
  QPushButton* cancel_path_btn_ = nullptr;
  QDoubleSpinBox* factor_spin_ = nullptr;
  QDoubleSpinBox* radius_spin_ = nullptr;
  QLineEdit* frame_input_ = nullptr;
  QComboBox* planner_combo_ = nullptr;
  QPushButton* refresh_planners_btn_ = nullptr;
  // planner name sent in GetPath goals; empty = MBF default (first loaded)
  std::string selected_planner_;
  // planner name restored from the rviz config before the list is available
  QString pending_planner_;
  // retries the 'planners' parameter query until move_base_flex is up
  QTimer* planner_poll_timer_ = nullptr;
  QPushButton* trace_back_btn_;
  QComboBox* planner_combo_ = nullptr;
  QPushButton* refresh_planners_btn_ = nullptr;
  // planner name sent in GetPath goals; empty = MBF default (first loaded)
  std::string selected_planner_;
  // planner name restored from the rviz config before the list is available
  QString pending_planner_;
  // retries the 'planners' parameter query until move_base_flex is up
  QTimer* planner_poll_timer_ = nullptr;

  // rclcpp node and parameter client
  rclcpp::Node::SharedPtr rcl_node_;
  std::shared_ptr<rclcpp::AsyncParametersClient> param_client_;
  // mapping server plugin namespace inside move_base_flex; owns the octomap
  // subscription and penalty-spread params (independent of planner names)
  std::string mapping_server_name_ = "octo_mapping_server";
  // planner node (kept for other parameter operations), and remote node to control
  std::string planner_node_name_ = "octo_planner";
  // mapping server plugin namespace inside move_base_flex; owns the octomap
  // subscription and penalty-spread params (independent of planner names)
  std::string mapping_server_name_ = "octo_mapping_server";
  // node and parameter key used for toggling octomap updates in the robot stack
  std::string param_node_name_ = "/move_base_flex";
  std::string param_key_ = "octo_mapping_server.enable_octomap_updates";
  bool octomap_enabled_ = true;
  QTimer* spin_timer_ = nullptr;
  // true if this panel called rclcpp::init()
  bool we_initialized_rclcpp_ = false;

  using GetPath = mbf_msgs::action::GetPath;
  rclcpp_action::Client<GetPath>::SharedPtr get_path_client_;

  // latched publisher: dropdown selection for exe_path_node's MoveBase goals
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planner_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string map_frame_ = "map";
  std::string get_path_action_name_ = "/move_base_flex/get_path";

  using StartNav = bring_up_alert_nav::srv::StartNav;
  rclcpp::Client<StartNav>::SharedPtr exec_path_client_;
  std::string exec_path_service_name_ = "/exe_path/start_nav";

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cancel_path_client_;
  std::string cancel_path_service_name_ = "/exe_path/cancel_nav";

  // Starts the autonomous frontier-exploration loop in exe_path_node.
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_explore_client_;
  std::string start_explore_service_name_ = "/exe_path/start_exploration";
  using ExploreToGoal = alert_msgs::action::ExploreToGoal;
  rclcpp_action::Client<ExploreToGoal>::SharedPtr explore_cancel_client_;
  std::string explore_action_name_ = "/graph_explorer/explore_to_goal";

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  std::string cmd_vel_topic_ = "cmd_vel";
  // Starts the autonomous frontier-exploration loop in exe_path_node.
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_explore_client_;
  std::string start_explore_service_name_ = "/exe_path/start_exploration";
};

} // namespace alert_nav_plugins
