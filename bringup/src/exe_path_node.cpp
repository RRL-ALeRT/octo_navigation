#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "bringup/srv/start_nav.hpp"
#include <mbf_msgs/action/exe_path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <mbf_msgs/action/move_base.hpp>
#include <mbf_msgs/action/get_path.hpp>

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
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose",      // RViz 預設 topic
        1,
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
    srv_ = create_service<bringup::srv::StartNav>(
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

  }

private:
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
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
      goal_msg.target_pose = *msg;

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
      goal_msg.target_pose = *msg;

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
  void onTrigger(const std::shared_ptr<bringup::srv::StartNav::Request> req,
                 std::shared_ptr<bringup::srv::StartNav::Response>      res)
  {
    RCLCPP_INFO(get_logger(), "Triggered Exe path");
    const nav_msgs::msg::Path *chosen = nullptr;
    switch (req->mode)
    {
      case 1: chosen = &latest_path_;    break;
      case 2: chosen = &latest_path__smooth_; break;
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

    // action goal
    mbf_msgs::action::ExePath::Goal goal;
    goal.path = *chosen;

    auto send_opts = rclcpp_action::Client<mbf_msgs::action::ExePath>::SendGoalOptions();
    send_opts.goal_response_callback =
        [this](auto handle) {                                         // log after send
          if(!handle)
            RCLCPP_INFO(get_logger(), "Nav finished SUCCESS");
          else
            RCLCPP_WARN(get_logger(), "Nav finished with outcome");
        };
    auto send =
      exe_path_ac_->async_send_goal(goal, send_opts);

    res->success = true;
    res->message = "ExePath sent";
  }

  /* ---------- members ---------- */
  nav_msgs::msg::Path latest_path_;
  nav_msgs::msg::Path latest_path__smooth_;

  // 2D Goal pose
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp_action::Client<mbf_msgs::action::MoveBase>::SharedPtr     mbf_movebase_ac_;
  rclcpp_action::Client<mbf_msgs::action::GetPath>::SharedPtr      mbf_getpath_ac_;
  // Exe path
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr smooth_sub_;
  rclcpp::Service<bringup::srv::StartNav>::SharedPtr srv_;
  rclcpp_action::Client<mbf_msgs::action::ExePath>::SharedPtr exe_path_ac_;
};

int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<ExePath>());
  rclcpp::shutdown();
  return 0;
}
