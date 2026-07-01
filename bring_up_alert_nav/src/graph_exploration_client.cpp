#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <alert_msgs/action/explore_to_goal.hpp>

namespace bring_up_alert_nav
{

class GraphExplorationClient : public rclcpp::Node
{
public:
  using ExploreToGoal = alert_msgs::action::ExploreToGoal;
  using GoalHandle    = rclcpp_action::ClientGoalHandle<ExploreToGoal>;

  GraphExplorationClient()
  : Node("graph_exploration_client")
  {
    action_name_ = declare_parameter("explore_action", std::string("/graph_explorer/explore_to_goal"));
    pose_topic_  = declare_parameter("pose_topic",     std::string("/alert_exploration_pose"));

    client_ = rclcpp_action::create_client<ExploreToGoal>(this, action_name_);

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, rclcpp::QoS(1),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { onPose(msg); });

    RCLCPP_INFO(get_logger(),
      "GraphExplorationClient ready. Listening on '%s', sending to '%s'.",
      pose_topic_.c_str(), action_name_.c_str());
  }

private:
  void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!client_->action_server_is_ready()) {
      RCLCPP_WARN(get_logger(), "Action server '%s' not ready — dropping goal.", action_name_.c_str());
      return;
    }

    // Cancel any currently running goal before sending the new one
    if (active_handle_) {
      RCLCPP_INFO(get_logger(), "New pose received — cancelling current exploration goal.");
      client_->async_cancel_goal(active_handle_);
      active_handle_ = nullptr;
    }

    ExploreToGoal::Goal goal;
    goal.target_pose = *msg;

    auto opts = rclcpp_action::Client<ExploreToGoal>::SendGoalOptions();

    opts.goal_response_callback = [this](GoalHandle::SharedPtr handle) {
      if (!handle) {
        RCLCPP_ERROR(get_logger(), "Explore goal rejected by server.");
        return;
      }
      RCLCPP_INFO(get_logger(), "Explore goal accepted.");
      active_handle_ = handle;
    };

    opts.feedback_callback = [this](
        GoalHandle::SharedPtr,
        const std::shared_ptr<const ExploreToGoal::Feedback> fb) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "Exploring — distance to goal: %.2f m", fb->distance_to_goal);
    };

    opts.result_callback = [this](const GoalHandle::WrappedResult& result) {
      active_handle_ = nullptr;
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(get_logger(),
            "Exploration succeeded. dist_to_goal=%.2f m, angle_to_goal=%.2f rad",
            result.result->dist_to_goal, result.result->angle_to_goal);
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_INFO(get_logger(), "Exploration cancelled.");
          break;
        default:
          RCLCPP_WARN(get_logger(),
            "Exploration aborted. dist_to_goal=%.2f m",
            result.result->dist_to_goal);
          break;
      }
    };

    RCLCPP_INFO(get_logger(),
      "Sending exploration goal: (%.2f, %.2f, %.2f)",
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    client_->async_send_goal(goal, opts);
  }

  rclcpp_action::Client<ExploreToGoal>::SharedPtr client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  GoalHandle::SharedPtr active_handle_;

  std::string action_name_;
  std::string pose_topic_;
};

}  // namespace bring_up_alert_nav

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bring_up_alert_nav::GraphExplorationClient>());
  rclcpp::shutdown();
  return 0;
}
