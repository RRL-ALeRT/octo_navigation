#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <mbf_msgs/action/get_path.hpp>
#include <mbf_msgs/action/exe_path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "astar_avoid_holes_octo_planner/srv/find_frontiers.hpp"

#include <vector>
#include <chrono>

using namespace std::chrono_literals;

using FindFrontiers = astar_avoid_holes_octo_planner::srv::FindFrontiers;
using GetPath       = mbf_msgs::action::GetPath;
using ExePath       = mbf_msgs::action::ExePath;

class ExplorationNode : public rclcpp::Node
{
public:
  ExplorationNode()
  : Node("exploration_node")
  {
    cb_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    this->declare_parameter("min_cluster_size",   1);
    this->declare_parameter("planner",            "");
    this->declare_parameter("controller",         ""); // leer = MBF-Default
    this->declare_parameter("loop_rate_sec",      2.0);

    min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
    planner_          = this->get_parameter("planner").as_string();
    controller_       = this->get_parameter("controller").as_string();
    loop_rate_sec_    = this->get_parameter("loop_rate_sec").as_double();

    frontier_client_ = this->create_client<FindFrontiers>("find_frontiers");

    get_path_client_ = rclcpp_action::create_client<GetPath>(
      this, "/move_base_flex/get_path");

    exe_path_client_ = rclcpp_action::create_client<ExePath>(
      this, "/move_base_flex/exe_path");

    RCLCPP_INFO(this->get_logger(),
      "exploration_node bereit. Planner: %s", planner_.c_str());

    // Exploration-Loop als Timer
      
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(loop_rate_sec_),
    std::bind(&ExplorationNode::exploration_step, this),
    cb_group_);  // ← Callback-Group übergeben
  }

private:
  // ---------------------------------------------------------------------------
  // Haupt-Loop
  // ---------------------------------------------------------------------------

  void exploration_step()
  {
    // Während einer laufenden Navigation nicht neu planen
    if (navigating_) return;

    RCLCPP_INFO(this->get_logger(), "Suche Frontiers...");

    // 1. Frontier Service aufrufen
    if (!frontier_client_->wait_for_service(2s)) {
      RCLCPP_WARN(this->get_logger(), "find_frontiers Service nicht erreichbar.");
      return;
    }

    auto request = std::make_shared<FindFrontiers::Request>();
    request->min_cluster_size = min_cluster_size_;

    auto future = frontier_client_->async_send_request(request);
    if (future.wait_for(20s) != std::future_status::ready) {
      RCLCPP_WARN(this->get_logger(), "find_frontiers Timeout.");
      return;
    }

    auto response = future.get();
    if (!response->success || response->frontier_centroids.empty()) {
      RCLCPP_INFO(this->get_logger(),
        "Keine Frontiers: %s", response->message.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(),
      "%zu Frontiers empfangen, probiere GetPath...",
      response->frontier_centroids.size());

    // 2. Jeden Frontier per GetPath validieren, erster erfolgreicher wird gefahren
    for (size_t i = 0; i < response->frontier_centroids.size(); ++i) {
      const auto& goal_pose = response->frontier_centroids[i];

      RCLCPP_INFO(this->get_logger(),
        "Probiere Frontier [%zu] bei (%.2f, %.2f)...",
        i + 1,
        goal_pose.pose.position.x,
        goal_pose.pose.position.y);

      auto path = try_get_path(goal_pose);
      if (path.empty()) {
        RCLCPP_WARN(this->get_logger(),
          "Frontier [%zu]: kein Plan gefunden, nächster...", i + 1);
        continue;
      }

      RCLCPP_INFO(this->get_logger(),
        "Frontier [%zu]: Plan gefunden (%zu Poses), starte Navigation.",
        i + 1, path.size());

      // 3. Gefundenen Pfad ausführen
      navigating_ = true;
      execute_path(path);
      return;
    }

    RCLCPP_WARN(this->get_logger(),
      "Kein planbarer Frontier gefunden.");
  }

  // ---------------------------------------------------------------------------
  // GetPath – gibt leeren Vektor zurück wenn kein Plan möglich
  // ---------------------------------------------------------------------------

  std::vector<geometry_msgs::msg::PoseStamped> try_get_path(
    const geometry_msgs::msg::PoseStamped& target)
  {
    if (!get_path_client_->wait_for_action_server(2s)) {
      RCLCPP_WARN(this->get_logger(), "GetPath Action Server nicht erreichbar.");
      return {};
    }

    auto goal = GetPath::Goal();
    goal.target_pose = target;
    goal.planner     = planner_;
    goal.tolerance   = 0.5;

    auto future = get_path_client_->async_send_goal(goal);
    if (future.wait_for(10s) != std::future_status::ready) {
      RCLCPP_WARN(this->get_logger(), "GetPath Timeout.");
      return {};
    }

    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_WARN(this->get_logger(), "GetPath Goal abgelehnt.");
      return {};
    }

    auto result_future = get_path_client_->async_get_result(goal_handle);
    if (result_future.wait_for(30s) != std::future_status::ready) {
      RCLCPP_WARN(this->get_logger(), "GetPath Result Timeout.");
      return {};
    }

    auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_WARN(this->get_logger(),
        "GetPath fehlgeschlagen (outcome=%d): %s",
        result.result->outcome, result.result->message.c_str());
      return {};
    }

    return result.result->path.poses;
  }

  // ---------------------------------------------------------------------------
  // ExePath – führt den übergebenen Pfad aus (async, navigating_ wird zurückgesetzt)
  // ---------------------------------------------------------------------------

  void execute_path(const std::vector<geometry_msgs::msg::PoseStamped>& poses)
  {
    if (!exe_path_client_->wait_for_action_server(2s)) {
      RCLCPP_WARN(this->get_logger(), "ExePath Action Server nicht erreichbar.");
      navigating_ = false;
      return;
    }

    nav_msgs::msg::Path path;
    path.header.frame_id = poses.empty() ? "map" : poses.front().header.frame_id;
    path.header.stamp    = this->now();
    path.poses           = poses;

    auto goal = ExePath::Goal();
    goal.path       = path;
    goal.controller = controller_;

    auto send_goal_options = rclcpp_action::Client<ExePath>::SendGoalOptions();

    send_goal_options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<ExePath>::WrappedResult& result) {
        navigating_ = false;
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(this->get_logger(), "Navigation erfolgreich.");
        } else {
          RCLCPP_WARN(this->get_logger(),
            "Navigation fehlgeschlagen (outcome=%d): %s",
            result.result->outcome, result.result->message.c_str());
        }
      };

    send_goal_options.feedback_callback =
      [this](rclcpp_action::ClientGoalHandle<ExePath>::SharedPtr,
             const std::shared_ptr<const ExePath::Feedback> feedback) {
        RCLCPP_DEBUG(this->get_logger(),
          "Navigation: dist_to_goal=%.2f", feedback->dist_to_goal);
      };

    exe_path_client_->async_send_goal(goal, send_goal_options);
  }

  // ---------------------------------------------------------------------------
  // Member
  // ---------------------------------------------------------------------------

  int         min_cluster_size_;
  std::string planner_;
  std::string controller_;
  double      loop_rate_sec_;
  bool        navigating_ = false;

  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Client<FindFrontiers>::SharedPtr              frontier_client_;
  rclcpp_action::Client<GetPath>::SharedPtr             get_path_client_;
  rclcpp_action::Client<ExePath>::SharedPtr             exe_path_client_;
  rclcpp::TimerBase::SharedPtr                          timer_;
};

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<ExplorationNode>();
  
  // MultiThreadedExecutor damit Service-Callbacks während wait_for laufen können
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
