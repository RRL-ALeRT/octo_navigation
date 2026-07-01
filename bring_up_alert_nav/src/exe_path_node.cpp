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

    // ⑤ Exploration loop: GetPath(frontier) → ExePath → repeat, until no reachable
    //    frontier is found max_explore_tries_ times in a row.
    max_explore_tries_ = this->declare_parameter("max_explore_tries", 6);
    explore_planner_   = this->declare_parameter("explore_planner", std::string("frontier"));
    start_explore_srv_ = create_service<std_srvs::srv::Trigger>(
      "/exe_path/start_exploration",
      std::bind(&ExePath::onStartExploration, this,
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
        [this](auto handle) {
          if(!handle) {
            RCLCPP_WARN(get_logger(), "ExePath goal rejected");
            exe_path_goal_handle_ = nullptr;
          } else {
            RCLCPP_INFO(get_logger(), "ExePath goal accepted");
            exe_path_goal_handle_ = handle;
          }
        };
    auto send =
      exe_path_ac_->async_send_goal(goal, send_opts);

    res->success = true;
    res->message = "ExePath sent";
  }

  // ---------------------------------------------------------------------------
  // Exploration loop
  // ---------------------------------------------------------------------------
  void onStartExploration(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (exploring_) {
      res->success = false;
      res->message = "Already exploring";
      return;
    }
    exploring_ = true;
    explore_fail_count_ = 0;
    RCLCPP_INFO(get_logger(), "Exploration started (planner='%s', max_tries=%d)",
                explore_planner_.c_str(), max_explore_tries_);
    res->success = true;
    res->message = "Exploration started";
    exploreStep();
  }

  void stopExploration(const std::string& why)
  {
    exploring_ = false;
    if (retry_timer_) retry_timer_->cancel();
    RCLCPP_INFO(get_logger(), "Exploration stopped: %s", why.c_str());
  }

  void scheduleExploreStep(std::chrono::milliseconds delay)
  {
    if (!exploring_) return;
    retry_timer_ = create_wall_timer(delay, [this]() {
      retry_timer_->cancel();
      exploreStep();
    });
  }

  // Ask the frontier planner for a path; on success drive it, else count the miss.
  void exploreStep()
  {
    if (!exploring_) return;

    if (!mbf_getpath_ac_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_WARN(get_logger(), "Explore: GetPath server not available; retrying");
      scheduleExploreStep(std::chrono::seconds(1));
      return;
    }

    // The frontier planner ignores the target pose; send a dummy in map frame.
    auto goal = mbf_msgs::action::GetPath::Goal();
    goal.use_start_pose = false;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = now();
    goal.target_pose.pose.orientation.w = 1.0;
    goal.planner = explore_planner_;
    goal.tolerance = 0.0;

    auto opts = rclcpp_action::Client<mbf_msgs::action::GetPath>::SendGoalOptions();
    opts.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<mbf_msgs::action::GetPath>::WrappedResult & res){
        if (!exploring_) return;
        const bool ok = res.code == rclcpp_action::ResultCode::SUCCEEDED
                        && res.result && !res.result->path.poses.empty();
        if (ok) {
          explore_fail_count_ = 0;
          RCLCPP_INFO(get_logger(), "Explore: frontier path (%zu poses) → execute",
                      res.result->path.poses.size());
          execExplorePath(res.result->path);
        } else {
          explore_fail_count_++;
          RCLCPP_WARN(get_logger(), "Explore: no reachable frontier (%d/%d): %s",
                      explore_fail_count_, max_explore_tries_,
                      res.result ? res.result->message.c_str() : "no result");
          if (explore_fail_count_ >= max_explore_tries_)
            stopExploration("no frontiers after max tries");
          else
            scheduleExploreStep(std::chrono::seconds(1));
        }
      };
    mbf_getpath_ac_->async_send_goal(goal, opts);
  }

  // Drive the frontier path; when the leg finishes (success or not) plan the next.
  void execExplorePath(const nav_msgs::msg::Path& path)
  {
    if (!exploring_) return;
    if (!exe_path_ac_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_WARN(get_logger(), "Explore: ExePath server not available; retrying plan");
      scheduleExploreStep(std::chrono::seconds(1));
      return;
    }

    mbf_msgs::action::ExePath::Goal goal;
    goal.path = path;

    auto opts = rclcpp_action::Client<mbf_msgs::action::ExePath>::SendGoalOptions();
    opts.goal_response_callback =
      [this](auto handle){ exe_path_goal_handle_ = handle; };
    opts.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<mbf_msgs::action::ExePath>::WrappedResult & res){
        exe_path_goal_handle_ = nullptr;
        if (!exploring_) return;
        RCLCPP_INFO(get_logger(), "Explore: nav leg finished (code=%d) → plan next",
                    static_cast<int>(res.code));
        // Give the octomap a moment to integrate what the robot just saw before we
        // replan, so the next frontier reflects the freshly-observed space.
        scheduleExploreStep(std::chrono::seconds(1));
      };
    exe_path_ac_->async_send_goal(goal, opts);
  }

  /* ---------- members ---------- */
  nav_msgs::msg::Path latest_path_;
  nav_msgs::msg::Path latest_path__smooth_;

  // Exploration loop state
  bool exploring_ = false;
  int  explore_fail_count_ = 0;
  int  max_explore_tries_ = 6;
  std::string explore_planner_ = "frontier";
  rclcpp::TimerBase::SharedPtr retry_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_explore_srv_;

  // Cancel service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;
  rclcpp_action::ClientGoalHandle<mbf_msgs::action::ExePath>::SharedPtr exe_path_goal_handle_;

  void onCancel(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    // Also stop the exploration loop so it doesn't immediately re-plan.
    if (exploring_) stopExploration("cancel requested");

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
