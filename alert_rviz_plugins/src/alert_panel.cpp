#include "alert_nav_plugins/alert_panel.hpp"
#include <rviz_common/config.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/exceptions.h>

namespace alert_nav_plugins
{

AlertPanel::AlertPanel(QWidget* parent)
: rviz_common::Panel(parent)
{
  auto v = new QVBoxLayout();

  toggle_btn_ = new QPushButton("Toggle Octomap Updates");
  v->addWidget(toggle_btn_);
  connect(toggle_btn_, &QPushButton::clicked, this, &AlertPanel::onToggleOctomap);

  auto h1 = new QHBoxLayout();
  h1->addWidget(new QLabel("Penalty spread factor"));
  factor_spin_ = new QDoubleSpinBox();
  factor_spin_->setRange(0.0, 1.0);
  factor_spin_->setSingleStep(0.01);
  factor_spin_->setValue(0.25);
  h1->addWidget(factor_spin_);
  connect(factor_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &AlertPanel::onFactorChanged);
  v->addLayout(h1);

  auto h2 = new QHBoxLayout();
  h2->addWidget(new QLabel("Penalty spread radius (m)"));
  radius_spin_ = new QDoubleSpinBox();
  radius_spin_->setRange(0.0, 5.0);
  radius_spin_->setSingleStep(0.01);
  radius_spin_->setValue(0.6);
  h2->addWidget(radius_spin_);
  connect(radius_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &AlertPanel::onRadiusChanged);
  v->addLayout(h2);

  auto hp = new QHBoxLayout();
  hp->addWidget(new QLabel("Planner"));
  planner_combo_ = new QComboBox();
  planner_combo_->addItem("(default)");
  planner_combo_->setToolTip(
    "Planner sent with GetPath goals. '(default)' lets move_base_flex use the "
    "first planner in its 'planners' list.");
  hp->addWidget(planner_combo_, 1);
  refresh_planners_btn_ = new QPushButton("⟳");
  refresh_planners_btn_->setFixedWidth(28);
  refresh_planners_btn_->setToolTip("Re-query loaded planners from move_base_flex");
  hp->addWidget(refresh_planners_btn_);
  connect(planner_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &AlertPanel::onPlannerChanged);
  connect(refresh_planners_btn_, &QPushButton::clicked, this, &AlertPanel::refreshPlannerList);
  v->addLayout(hp);

  auto h3 = new QHBoxLayout();
  h3->addWidget(new QLabel("Target frame"));
  frame_input_ = new QLineEdit();
  frame_input_->setPlaceholderText("e.g. Linear_Inspect_KRail");
  h3->addWidget(frame_input_);
  plan_btn_ = new QPushButton("Plan path");
  h3->addWidget(plan_btn_);
  connect(plan_btn_, &QPushButton::clicked, this, &AlertPanel::onPlanToFrame);
  v->addLayout(h3);

  explore_btn_ = new QPushButton("Explore (frontier)");
  explore_btn_->setStyleSheet("color: white; background-color: #2980b9;");
  v->addWidget(explore_btn_);
  connect(explore_btn_, &QPushButton::clicked, this, &AlertPanel::onExplore);

  exec_path_btn_ = new QPushButton("Exec Path");
  v->addWidget(exec_path_btn_);
  connect(exec_path_btn_, &QPushButton::clicked, this, &AlertPanel::onExecPath);

  cancel_path_btn_ = new QPushButton("Cancel Path/Exp");
  cancel_path_btn_->setStyleSheet("color: white; background-color: #c0392b;");
  v->addWidget(cancel_path_btn_);
  connect(cancel_path_btn_, &QPushButton::clicked, this, &AlertPanel::onCancelPath);

  setLayout(v);

  // create rcl node for parameter updates
  try {
    rclcpp::init(0, nullptr);
    we_initialized_rclcpp_ = true;
  } catch (const std::exception & e) {
    // already initialized by rviz / parent process
    we_initialized_rclcpp_ = false;
  }
  rcl_node_ = std::make_shared<rclcpp::Node>("alert_nav_rvizpanel_node");
  param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(rcl_node_, param_node_name_);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(rcl_node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, rcl_node_, false);
  get_path_client_ = rclcpp_action::create_client<GetPath>(rcl_node_, get_path_action_name_);
  // Latched so exe_path_node gets the selection even if it starts later; it
  // sends the planner with the MoveBase goals triggered by 2D Pose Estimate.
  planner_pub_ = rcl_node_->create_publisher<std_msgs::msg::String>(
    "/alert_panel/selected_planner", rclcpp::QoS(1).transient_local());
  exec_path_client_ = rcl_node_->create_client<StartNav>(exec_path_service_name_);
  cancel_path_client_ = rcl_node_->create_client<std_srvs::srv::Trigger>(cancel_path_service_name_);
  explore_cancel_client_ = rclcpp_action::create_client<ExploreToGoal>(rcl_node_, explore_action_name_);
  cmd_vel_pub_ = rcl_node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, rclcpp::QoS(1));

  // Log so we can see in the rviz terminal that the panel was constructed
  try {
    RCLCPP_INFO(rcl_node_->get_logger(), "AlertPanel constructed and parameter client created for '%s'", param_node_name_.c_str());
  } catch(...) {
    // fallback to cerr if logging subsystem isn't fully up
    std::cerr << "AlertPanel constructed for planner: " << param_node_name_ << std::endl;
  }

  // periodically spin the rcl node in the background to process calls
  spin_timer_ = new QTimer(this);
  connect(spin_timer_, &QTimer::timeout, [this]() { rclcpp::spin_some(rcl_node_); });
  spin_timer_->start(100);

  // keep asking for the loaded planner list until move_base_flex answers
  planner_poll_timer_ = new QTimer(this);
  connect(planner_poll_timer_, &QTimer::timeout, this, &AlertPanel::refreshPlannerList);
  planner_poll_timer_->start(3000);
  // read current parameter value asynchronously so we don't block the GUI thread
  std::thread([this]() {
    try {
      auto fut = param_client_->get_parameters({param_key_});
      if (fut.valid()) {
        auto vec = fut.get();
        if (!vec.empty() && vec[0].get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
          octomap_enabled_ = vec[0].as_bool();
        }
      }
    } catch(...) {}
    // update UI on GUI thread
    QMetaObject::invokeMethod(this, "updateButtonUI", Qt::QueuedConnection, Q_ARG(bool, octomap_enabled_));
  }).detach();
}

AlertPanel::~AlertPanel()
{
  try {
    if (we_initialized_rclcpp_) rclcpp::shutdown();
  } catch(...) {}
}

void AlertPanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
  QString planner;
  if (config.mapGetString("planner", &planner) && !planner.isEmpty()) {
    const int idx = planner_combo_ ? planner_combo_->findText(planner) : -1;
    if (idx >= 0) {
      planner_combo_->setCurrentIndex(idx);
    } else {
      pending_planner_ = planner;
    }
  }
}

void AlertPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  if (planner_combo_) {
    config.mapSetValue("planner", planner_combo_->currentText());
  }
}

void AlertPanel::onToggleOctomap()
{
  if (!param_client_) return;
  octomap_enabled_ = !octomap_enabled_;
  std::vector<rclcpp::Parameter> params;
  params.emplace_back(param_key_, octomap_enabled_);
  param_client_->set_parameters(params);
  // update UI immediately
  updateButtonUI(octomap_enabled_);
}

void AlertPanel::onFactorChanged(double v)
{
  if (!param_client_) return;
  std::vector<rclcpp::Parameter> params;
  params.emplace_back(mapping_server_name_ + ".penalty_spread_factor", v);
  params.emplace_back(mapping_server_node_name_ + ".penalty_spread_factor", v);
  param_client_->set_parameters(params);
}

void AlertPanel::onRadiusChanged(double v)
{
  if (!param_client_) return;
  std::vector<rclcpp::Parameter> params;
  params.emplace_back(mapping_server_name_ + ".penalty_spread_radius", v);
  params.emplace_back(mapping_server_node_name_ + ".penalty_spread_radius", v);
  param_client_->set_parameters(params);
}

void AlertPanel::onPlannerChanged(int index)
{
  selected_planner_ = (index <= 0 || !planner_combo_)
    ? std::string()
    : planner_combo_->itemText(index).toStdString();
  if (planner_pub_) {
    std_msgs::msg::String msg;
    msg.data = selected_planner_;
    planner_pub_->publish(msg);
  }
  if (rcl_node_) {
    RCLCPP_INFO(rcl_node_->get_logger(), "GetPath planner set to '%s'",
                selected_planner_.empty() ? "(default)" : selected_planner_.c_str());
  }
}

void AlertPanel::refreshPlannerList()
{
  if (!param_client_ || !param_client_->service_is_ready()) {
    return;
  }
  param_client_->get_parameters(
    {"planners"},
    [this](std::shared_future<std::vector<rclcpp::Parameter>> future) {
      QStringList names;
      try {
        auto params = future.get();
        if (!params.empty() && params[0].get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
          for (const auto& name : params[0].as_string_array()) {
            names << QString::fromStdString(name);
          }
        }
      } catch (...) {}
      // hop to the GUI thread before touching widgets
      QMetaObject::invokeMethod(this, "updatePlannerList", Qt::QueuedConnection,
                                Q_ARG(QStringList, names));
    });
}

void AlertPanel::updatePlannerList(QStringList names)
{
  if (!planner_combo_ || names.isEmpty()) {
    return;
  }
  if (planner_poll_timer_) {
    planner_poll_timer_->stop();
  }

  const QString wanted = !pending_planner_.isEmpty() ? pending_planner_ : planner_combo_->currentText();
  planner_combo_->blockSignals(true);
  planner_combo_->clear();
  planner_combo_->addItem("(default)");
  planner_combo_->addItems(names);
  const int idx = planner_combo_->findText(wanted);
  planner_combo_->setCurrentIndex(idx >= 0 ? idx : 0);
  planner_combo_->blockSignals(false);
  pending_planner_.clear();
  onPlannerChanged(planner_combo_->currentIndex());

  RCLCPP_INFO(rcl_node_->get_logger(), "Loaded planners from move_base_flex: %s",
              names.join(", ").toStdString().c_str());
}

void AlertPanel::onPlanToFrame()
{
  if (!frame_input_ || !rcl_node_)
  {
    return;
  }

  const auto frame_name = frame_input_->text().trimmed().toStdString();
  if (frame_name.empty())
  {
    RCLCPP_WARN(rcl_node_->get_logger(), "No target frame provided for GetPath request");
    return;
  }

  if (!tf_buffer_)
  {
    RCLCPP_ERROR(rcl_node_->get_logger(), "TF buffer not initialized");
    return;
  }

  geometry_msgs::msg::TransformStamped tf_map_target;
  try
  {
    tf_map_target = tf_buffer_->lookupTransform(map_frame_, frame_name, tf2::TimePointZero);
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_WARN(
      rcl_node_->get_logger(),
      "Failed to lookup transform map->%s: %s",
      frame_name.c_str(),
      ex.what());
    return;
  }

  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = map_frame_;
  target_pose.header.stamp = rcl_node_->now();
  target_pose.pose.position.x = tf_map_target.transform.translation.x;
  target_pose.pose.position.y = tf_map_target.transform.translation.y;
  target_pose.pose.position.z = tf_map_target.transform.translation.z;
  target_pose.pose.orientation = tf_map_target.transform.rotation;

  if (!get_path_client_)
  {
    RCLCPP_ERROR(rcl_node_->get_logger(), "GetPath action client not initialized");
    return;
  }

  if (!get_path_client_->action_server_is_ready())
  {
    RCLCPP_WARN(
      rcl_node_->get_logger(),
      "GetPath action server '%s' is not ready",
      get_path_action_name_.c_str());
    return;
  }

  GetPath::Goal goal;
  goal.use_start_pose = false;
  goal.target_pose = target_pose;
  goal.tolerance = 0.0;
  goal.planner = selected_planner_;
  goal.concurrency_slot = 0;

  auto options = rclcpp_action::Client<GetPath>::SendGoalOptions();
  options.goal_response_callback = [logger = rcl_node_->get_logger(), frame_name](auto handle) {
    if (!handle)
    {
      RCLCPP_ERROR(logger, "GetPath goal rejected for frame '%s'", frame_name.c_str());
    }
    else
    {
      RCLCPP_INFO(logger, "GetPath goal accepted for frame '%s'", frame_name.c_str());
    }
  };
  options.result_callback = [logger = rcl_node_->get_logger(), frame_name](const auto &result) {
    if (!result.result)
    {
      RCLCPP_ERROR(logger, "GetPath result missing for frame '%s'", frame_name.c_str());
      return;
    }
    RCLCPP_INFO(
      logger,
      "GetPath completed for frame '%s' (outcome=%d): %s",
      frame_name.c_str(),
      result.result->outcome,
      result.result->message.c_str());
  };

  get_path_client_->async_send_goal(goal, options);
}

void AlertPanel::onExplore()
{
  // Kick off the autonomous exploration loop in exe_path_node. It repeatedly asks
  // the frontier planner for a path and drives it, stopping after N consecutive
  // "no reachable frontier" results. Use "Cancel Path" to stop early.
  if (!start_explore_client_) {
    RCLCPP_ERROR(rcl_node_->get_logger(), "Start-exploration service client not initialized");
    return;
  }
  if (!start_explore_client_->service_is_ready()) {
    RCLCPP_WARN(rcl_node_->get_logger(), "Service '%s' is not ready",
                start_explore_service_name_.c_str());
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  start_explore_client_->async_send_request(
    request,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      try {
        auto response = future.get();
        if (response->success)
          RCLCPP_INFO(rcl_node_->get_logger(), "Exploration started: %s", response->message.c_str());
        else
          RCLCPP_WARN(rcl_node_->get_logger(), "Exploration not started: %s", response->message.c_str());
      } catch (const std::exception &e) {
        RCLCPP_ERROR(rcl_node_->get_logger(), "Start-exploration call failed: %s", e.what());
      }
    });
}

void AlertPanel::updateButtonUI(bool enabled)
{
  if (!toggle_btn_) return;
  if (enabled) {
    toggle_btn_->setText("Octomap updates ON");
    toggle_btn_->setStyleSheet("color: white; background-color: green;");
  } else {
    toggle_btn_->setText("Octomap updates OFF");
    toggle_btn_->setStyleSheet("color: white; background-color: red;");
  }
}

void AlertPanel::onExecPath()
{
  if (!exec_path_client_) {
    RCLCPP_ERROR(rcl_node_->get_logger(), "Exec Path service client not initialized");
    return;
  }
  if (!exec_path_client_->service_is_ready()) {
    RCLCPP_WARN(rcl_node_->get_logger(), "Service '%s' is not ready", exec_path_service_name_.c_str());
    return;
  }

  auto request = std::make_shared<StartNav::Request>();
  request->mode = 1;  // raw mode

  exec_path_client_->async_send_request(
    request,
    [this](rclcpp::Client<StartNav>::SharedFuture future) {
      try {
        auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(rcl_node_->get_logger(), "ExecPath succeeded: %s", response->message.c_str());
        } else {
          RCLCPP_WARN(rcl_node_->get_logger(), "ExecPath failed: %s", response->message.c_str());
        }
      } catch (const std::exception &e) {
        RCLCPP_ERROR(rcl_node_->get_logger(), "ExecPath service call failed: %s", e.what());
      }
    });
}

void AlertPanel::onCancelPath()
{
  // 1. Publish zero velocity immediately so the robot stops
  if (cmd_vel_pub_) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
    RCLCPP_INFO(rcl_node_->get_logger(), "Published zero cmd_vel.");
  }

  // 2. Cancel all active exploration goals
  if (explore_cancel_client_) {
    explore_cancel_client_->async_cancel_all_goals(
      [this](std::shared_ptr<action_msgs::srv::CancelGoal_Response> response) {
        RCLCPP_INFO(rcl_node_->get_logger(),
          "Exploration cancel response: %d goal(s) cancelled.",
          static_cast<int>(response->goals_canceling.size()));
      });
  } else {
    RCLCPP_WARN(rcl_node_->get_logger(), "Explore action client not initialized.");
  }

  // 3. Cancel the active nav path (existing service call)
  if (!cancel_path_client_) {
    RCLCPP_ERROR(rcl_node_->get_logger(), "Cancel Path service client not initialized");
    return;
  }
  if (!cancel_path_client_->service_is_ready()) {
    RCLCPP_WARN(rcl_node_->get_logger(), "Service '%s' is not ready", cancel_path_service_name_.c_str());
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  cancel_path_client_->async_send_request(
    request,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      try {
        auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(rcl_node_->get_logger(), "CancelPath succeeded: %s", response->message.c_str());
        } else {
          RCLCPP_WARN(rcl_node_->get_logger(), "CancelPath failed: %s", response->message.c_str());
        }
      } catch (const std::exception &e) {
        RCLCPP_ERROR(rcl_node_->get_logger(), "CancelPath service call failed: %s", e.what());
      }
    });
}

} // namespace alert_nav_plugins

PLUGINLIB_EXPORT_CLASS(alert_nav_plugins::AlertPanel, rviz_common::Panel);
