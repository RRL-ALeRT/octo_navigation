#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "bring_up_alert_nav/srv/start_nav.hpp"
#include <mbf_msgs/action/exe_path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <mbf_msgs/action/move_base.hpp>
#include <mbf_msgs/action/get_path.hpp>
#include <algorithm> 

#define auto_execute_goal_ 0

class ExePath : public rclcpp::Node
{
public:
  ExePath()
  : Node("exe_path")
  {
    /* ----------------------------------------------------------
    * RViz 2D Goal Pose  →  MBF MoveBase
    * ---------------------------------------------------------- */
    // Action client
    mbf_movebase_ac_ = rclcpp_action::create_client<mbf_msgs::action::MoveBase>(
        get_node_base_interface(), get_node_graph_interface(),
        get_node_logging_interface(), get_node_waitables_interface(),
        "move_base_flex/move_base");
    mbf_getpath_ac_ = rclcpp_action::create_client<mbf_msgs::action::GetPath>(
        get_node_base_interface(), get_node_graph_interface(),
        get_node_logging_interface(), get_node_waitables_interface(),
        "move_base_flex/get_path");

    // Subscriber
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose",      // RViz 預設 topic
        10,
        std::bind(&ExePath::goalPoseCallback, this, std::placeholders::_1));

    // ① Get topic of path（astar_2D_planner's publisher）
    std::string path_topic =
      this->declare_parameter("path_topic",
                              "/move_base_flex/path");
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic, 10,
      [this](const nav_msgs::msg::Path::SharedPtr msg){
        latest_path_ = *msg;
        RCLCPP_INFO(get_logger(), "Got new path with %zu poses", msg->poses.size());
      });


    std::string path_topic_smooth =
      this->declare_parameter("path_topic_smooth",
                              "/move_base_flex/path_smooth");
    smooth_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_smooth, 10,
      [this](const nav_msgs::msg::Path::SharedPtr msg){
        latest_path__smooth_ = *msg;
        RCLCPP_INFO(get_logger(), "Got new smooth path with %zu poses", msg->poses.size());
      });

    // ② set service   /exe_path/start_nav
    srv_ = create_service<bring_up_alert_nav::srv::StartNav>(
      "/exe_path/start_nav",
      std::bind(&ExePath::onTrigger, this,
                std::placeholders::_1, std::placeholders::_2));

    // ③ ExePath action client
    exe_path_ac_ = rclcpp_action::create_client<mbf_msgs::action::ExePath>(
      get_node_base_interface(),         // NodeBase
      get_node_graph_interface(),        // NodeGraph
      get_node_logging_interface(),      // NodeLogger
      get_node_waitables_interface(),    // NodeWaitables
      "move_base_flex/exe_path");

    // ④ Cancel service   /exe_path/cancel_nav
    cancel_srv_ = create_service<std_srvs::srv::Trigger>(
      "/exe_path/cancel_nav",
      std::bind(&ExePath::onCancel, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "ExePath Node ready - waiting for goal_pose.");
  }

private:
  void goalPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    if (auto_execute_goal_)
    {
      RCLCPP_INFO(get_logger(), "RViz goal received → send MoveBase");

      // wait MoveBase action server
      if (!mbf_movebase_ac_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(get_logger(), "MoveBase action server not available!");
        return;
      }

      auto goal_msg = mbf_msgs::action::MoveBase::Goal();
      goal_msg.target_pose.header = msg->header;
      goal_msg.target_pose.pose = msg->pose.pose;
      auto opts = rclcpp_action::Client<mbf_msgs::action::MoveBase>::SendGoalOptions();
      opts.goal_response_callback =
          [this](auto handle){
            if(!handle)
              RCLCPP_ERROR(get_logger(), "MoveBase goal rejected");
            else
              RCLCPP_INFO (get_logger(), "MoveBase goal accepted");
          };
      opts.result_callback =
          [this](const rclcpp_action::ClientGoalHandle<mbf_msgs::action::MoveBase>::WrappedResult & res){
            switch(res.code){
              case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO (get_logger(), "MoveBase succeeded");
                break;
              case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_WARN (get_logger(), "MoveBase aborted: %s", res.result->message.c_str());
                break;
              case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO (get_logger(), "MoveBase canceled");
                break;
              default:
                RCLCPP_ERROR(get_logger(), "MoveBase finished with unknown code");
            }
          };

      mbf_movebase_ac_->async_send_goal(goal_msg, opts);
    }
    else
    {
      RCLCPP_INFO(get_logger(), "RViz goal received → send GetPath");

      // wait GetPath action server
      if (!mbf_getpath_ac_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(get_logger(), "GetPath action server not available!");
        return;
      }

      auto goal_msg = mbf_msgs::action::GetPath::Goal();
      goal_msg.target_pose.header = msg->header;
      goal_msg.target_pose.pose = msg->pose.pose;

      auto opts = rclcpp_action::Client<mbf_msgs::action::GetPath>::SendGoalOptions();
      opts.goal_response_callback =
          [this](auto handle){
            if(!handle)
              RCLCPP_ERROR(get_logger(), "GetPath goal rejected");
            else
              RCLCPP_INFO (get_logger(), "GetPath goal accepted");
          };
      opts.result_callback =
          [this](const rclcpp_action::ClientGoalHandle<mbf_msgs::action::GetPath>::WrappedResult & res){
            switch(res.code){
              case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO (get_logger(), "GetPath succeeded");
                break;
              case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_WARN (get_logger(), "GetPath aborted: %s", res.result->message.c_str());
                break;
              case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO (get_logger(), "GetPath canceled");
                break;
              default:
                RCLCPP_ERROR(get_logger(), "GetPath finished with unknown code");
            }
          };

      mbf_getpath_ac_->async_send_goal(goal_msg, opts);
    }
  }

  // Service callback
  void onTrigger(const std::shared_ptr<bring_up_alert_nav::srv::StartNav::Request> req,
                 std::shared_ptr<bring_up_alert_nav::srv::StartNav::Response>      res)
  {
    RCLCPP_INFO(get_logger(), "Triggered Exe path");
    const nav_msgs::msg::Path *chosen = nullptr;
    bool is_reverse = false;

    switch (req->mode)
    {
      case 1: chosen = &latest_path_; break;
      case 2: chosen = &latest_path__smooth_; break;
      case 3: chosen = &latest_path_; is_reverse = true; break;
      default:
        res->success = false;
        res->message = "data=false -> ignore";
        return;
    }

    if(chosen->poses.empty()){
      res->success = false;
      res->message = "No path available yet";
      return;
    }

    if(!exe_path_ac_->wait_for_action_server(std::chrono::seconds(2))){
      res->success = false;
      res->message = "ExePath action server not ready";
      return;
    }

    // 1. Copy the path so we can modify it
    nav_msgs::msg::Path final_path = *chosen;

    // 2. If Tracing Back, reverse the array of waypoints
    if (is_reverse) {
        std::reverse(final_path.poses.begin(), final_path.poses.end());
    }

    // 3. Freshen timestamps 
    rclcpp::Time current_time = this->now();
    final_path.header.stamp = current_time;
    for (auto & pose : final_path.poses) {
        pose.header.stamp = current_time;
    }

    // action goal
    mbf_msgs::action::ExePath::Goal goal;
    goal.path = final_path;

    auto send_opts = rclcpp_action::Client<mbf_msgs::action::ExePath>::SendGoalOptions();
    send_opts.goal_response_callback =
        [this](auto handle) {
          if(!handle) {
            RCLCPP_WARN(get_logger(), "ExePath goal rejected");
            exe_path_goal_handle_ = nullptr;
          } else {
            RCLCPP_INFO(get_logger(), "ExePath goal accepted");
            exe_path_goal_handle_ = handle;
          }
        };
    
    exe_path_ac_->async_send_goal(goal, send_opts);

    res->success = true;
    res->message = is_reverse ? "Tracing back to start" : "ExePath sent";
  }

  /* ---------- members ---------- */
  nav_msgs::msg::Path latest_path_;
  nav_msgs::msg::Path latest_path__smooth_;

  // Cancel service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;
  rclcpp_action::ClientGoalHandle<mbf_msgs::action::ExePath>::SharedPtr exe_path_goal_handle_;

  void onCancel(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (!exe_path_goal_handle_) {
      res->success = false;
      res->message = "No active ExePath goal to cancel";
      RCLCPP_WARN(get_logger(), "Cancel requested but no active goal");
      return;
    }
    RCLCPP_INFO(get_logger(), "Cancelling ExePath goal...");
    exe_path_ac_->async_cancel_goal(exe_path_goal_handle_);
    exe_path_goal_handle_ = nullptr;
    res->success = true;
    res->message = "Cancel request sent";
  }

  // 2D Goal pose
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr goal_sub_;
  rclcpp_action::Client<mbf_msgs::action::MoveBase>::SharedPtr     mbf_movebase_ac_;
  rclcpp_action::Client<mbf_msgs::action::GetPath>::SharedPtr      mbf_getpath_ac_;
  // Exe path
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr smooth_sub_;
  rclcpp::Service<bring_up_alert_nav::srv::StartNav>::SharedPtr srv_;
  rclcpp_action::Client<mbf_msgs::action::ExePath>::SharedPtr exe_path_ac_;
};

int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<ExePath>());
  rclcpp::shutdown();
  return 0;
}