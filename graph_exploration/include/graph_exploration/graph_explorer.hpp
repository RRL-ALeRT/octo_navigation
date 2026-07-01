#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <mbf_msgs/action/move_base.hpp>
#include <alert_msgs/action/explore_to_goal.hpp>

#include <octomap/OcTree.h>

namespace graph_exploration
{

class GraphExplorer : public rclcpp::Node
{
public:
  GraphExplorer();
  ~GraphExplorer();

private:
  // --- Action type aliases ----------------------------------------------------
  using MoveBase        = mbf_msgs::action::MoveBase;
  using MBFGoalHandle   = rclcpp_action::ClientGoalHandle<MoveBase>;
  using ExploreToGoal   = alert_msgs::action::ExploreToGoal;
  using ExploreHandle   = rclcpp_action::ServerGoalHandle<ExploreToGoal>;

  // --- Explore action server --------------------------------------------------
  rclcpp_action::Server<ExploreToGoal>::SharedPtr explore_server_;
  std::shared_ptr<ExploreHandle>                  active_explore_handle_;
  std::mutex                                       explore_handle_mutex_;

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const ExploreToGoal::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<ExploreHandle> handle);

  void handleAccepted(const std::shared_ptr<ExploreHandle> handle);

  // --- Subscriptions ----------------------------------------------------------
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr   graph_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr      octomap_sub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ray_marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // --- TF ---------------------------------------------------------------------
  tf2_ros::Buffer            tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // --- MBF action client ------------------------------------------------------
  rclcpp_action::Client<MoveBase>::SharedPtr move_base_client_;

  std::shared_ptr<MBFGoalHandle> active_goal_handle_;
  std::mutex                     goal_handle_mutex_;

  // --- Graph node cache (protected by graph_mutex_) ---------------------------
  struct GraphNode {
    float x, y, z;
    float penalty;
    float neighbor_count;
  };
  std::vector<GraphNode> graph_nodes_;
  std::mutex             graph_mutex_;

  // --- Octomap cache (protected by octree_mutex_) -----------------------------
  std::shared_ptr<octomap::OcTree> octree_;
  std::mutex                        octree_mutex_;

  // --- Goal state (protected by goal_mutex_) ----------------------------------
  std::mutex                         goal_mutex_;
  std::condition_variable            goal_cv_;
  geometry_msgs::msg::PoseStamped    final_goal_;
  bool                               has_goal_{false};
  std::atomic_bool                   goal_changed_{false};
  std::atomic_bool                   shutdown_{false};

  // --- Exploration thread -----------------------------------------------------
  std::thread exploration_thread_;

  // --- Parameters -------------------------------------------------------------
  double      frontier_radius_;
  double      min_waypoint_dist_;
  double      goal_tolerance_;
  double      penalty_weight_;
  double      visited_radius_;
  double      candidate_penalty_ratio_;  // exclude nodes above this fraction of max_penalty
  double      frontier_max_neighbors_;   // neighbor_count threshold for "edge of map" nodes
  double      stuck_timeout_;            // seconds without movement before recovery (default 6)
  double      stuck_move_threshold_;     // meters robot must move to reset stuck timer (default 0.05)
  double      stuck_backup_dist_;        // meters to reverse (default 0.3)
  std::string robot_frame_;
  std::string map_frame_;
  std::string octomap_topic_;

  // --- Visited waypoint blacklist (cleared on new goal) ----------------------
  struct VisitedWaypoint { float x, y, z; };
  std::vector<VisitedWaypoint> visited_waypoints_;   // short-range (visited_radius_)
  std::mutex                   visited_mutex_;

  // --- Explored-frontier store (cleared on new goal) -------------------------
  // Wider exclusion zone: prevents re-exploring the same area.
  std::vector<VisitedWaypoint> explored_frontiers_;
  std::mutex                   explored_mutex_;
  double                       explored_frontier_radius_;

  // --- Callbacks --------------------------------------------------------------
  void onGraphCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg);

  // --- Exploration ------------------------------------------------------------
  void explorationLoop();
  void explore(const std::shared_ptr<ExploreHandle>& handle);
  bool getRobotPose(geometry_msgs::msg::PoseStamped& pose);

  // --- Candidate selection ----------------------------------------------------
  struct Candidate {
    float x, y, z;
    float penalty;
    float neighbor_count;
    float clearance;   // min-directional raycast clearance [0, 2.0m]
  };

  std::vector<Candidate> findCandidates(
    float robot_x, float robot_y, float robot_z,
    float robot_yaw,
    float goal_x,  float goal_y,
    const std::shared_ptr<octomap::OcTree>& tree);

  // Cast 5 robot-relative rays (front-left, front-right, left, right, back)
  // from 2 origins (base_link and rz-0.2).  Score = 0.6*clearance + 0.4*goal_align.
  // Publishes all 10 rays as a pink MarkerArray on ~/exploration_rays.
  std::pair<float,float> computeRobotClearanceDir(
    float rx, float ry, float rz,
    float robot_yaw,
    float goal_x, float goal_y,
    const octomap::OcTree& tree);

  // Cast 4 horizontal rays at 3 heights (0.5, 0.7, 0.9 m above node),
  // each up to kRayMaxDist. Returns minimum per-direction clearance.
  float computeClearance(const octomap::OcTree& tree,
                          float x, float y, float z) const;

  Candidate selectBest(
    const std::vector<Candidate>& candidates,
    float robot_x, float robot_y,
    float goal_x,  float goal_y);

  // --- Navigation -------------------------------------------------------------
  bool sendMoveBaseGoal(const geometry_msgs::msg::PoseStamped& target);
  void cancelCurrentGoal();
  void driveBackward(double dist_m, double speed_mps = 0.15);

  geometry_msgs::msg::PoseStamped makePoseToward(
    float px, float py, float pz,
    float tx, float ty);

  static constexpr float kRayMaxDist      = 2.0f;   // per-candidate clearance rays
  static constexpr float kRobotRayMaxDist = 5.0f;   // robot direction-finding rays
  static constexpr float kRayHeights[]    = {0.5f, 0.7f, 0.9f};
};

}  // namespace graph_exploration
