#include "graph_exploration/graph_explorer.hpp"

#include <algorithm>
#include <cmath>
#include <future>
#include <limits>

#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <octomap_msgs/octomap_msgs/conversions.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace graph_exploration
{

// constexpr array definition (required outside the class in C++14/17)
constexpr float GraphExplorer::kRayHeights[];

// =============================================================================
// Construction / destruction
// =============================================================================

GraphExplorer::GraphExplorer()
: Node("graph_explorer"),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_)
{
  frontier_radius_         = declare_parameter("frontier_radius",         2.0);
  min_waypoint_dist_       = declare_parameter("min_waypoint_dist",       0.7);
  goal_tolerance_          = declare_parameter("goal_tolerance",           0.3);
  penalty_weight_          = declare_parameter("penalty_weight",           0.4);
  visited_radius_          = declare_parameter("visited_radius",           0.7);
  candidate_penalty_ratio_   = declare_parameter("candidate_penalty_ratio",   0.65);
  frontier_max_neighbors_    = declare_parameter("frontier_max_neighbors",    4.0);
  explored_frontier_radius_  = declare_parameter("explored_frontier_radius",  .7);
  stuck_timeout_             = declare_parameter("stuck_timeout",             6.0);
  stuck_move_threshold_      = declare_parameter("stuck_move_threshold",      0.05);
  stuck_backup_dist_         = declare_parameter("stuck_backup_dist",         0.3);
  robot_frame_             = declare_parameter("robot_frame",              std::string("base_link"));
  map_frame_               = declare_parameter("map_frame",               std::string("map"));
  octomap_topic_           = declare_parameter("octomap_topic",            std::string("/octomap_binary"));

  explore_server_ = rclcpp_action::create_server<ExploreToGoal>(
    this,
    "~/explore_to_goal",
    [this](const rclcpp_action::GoalUUID& uuid,
           std::shared_ptr<const ExploreToGoal::Goal> goal) {
      return handleGoal(uuid, goal);
    },
    [this](const std::shared_ptr<ExploreHandle> handle) {
      return handleCancel(handle);
    },
    [this](const std::shared_ptr<ExploreHandle> handle) {
      handleAccepted(handle);
    });

  graph_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/move_base_flex/graph_cloud",
    rclcpp::QoS(1).transient_local(),
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { onGraphCloud(msg); });

  octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
    octomap_topic_, rclcpp::QoS(1).best_effort(),
    [this](const octomap_msgs::msg::Octomap::SharedPtr msg) { onOctomap(msg); });

  ray_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/exploration_rays", rclcpp::QoS(1));

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(1));

  move_base_client_ = rclcpp_action::create_client<MoveBase>(this, "move_base_flex/move_base");

  exploration_thread_ = std::thread(&GraphExplorer::explorationLoop, this);

  RCLCPP_INFO(get_logger(),
    "GraphExplorer ready. frontier_radius=%.1fm min_waypoint_dist=%.2fm "
    "goal_tolerance=%.2fm penalty_weight=%.2f visited_radius=%.2fm octomap='%s'",
    frontier_radius_, min_waypoint_dist_, goal_tolerance_,
    penalty_weight_, visited_radius_, octomap_topic_.c_str());
}

GraphExplorer::~GraphExplorer()
{
  shutdown_     = true;
  goal_changed_ = true;
  cancelCurrentGoal();
  goal_cv_.notify_all();
  if (exploration_thread_.joinable()) {
    exploration_thread_.join();
  }
}

// =============================================================================
// Subscription callbacks
// =============================================================================

rclcpp_action::GoalResponse GraphExplorer::handleGoal(
  const rclcpp_action::GoalUUID&,
  std::shared_ptr<const ExploreToGoal::Goal> goal)
{
  RCLCPP_INFO(get_logger(), "Exploration goal received: (%.2f, %.2f, %.2f)",
    goal->target_pose.pose.position.x,
    goal->target_pose.pose.position.y,
    goal->target_pose.pose.position.z);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GraphExplorer::handleCancel(
  const std::shared_ptr<ExploreHandle>)
{
  RCLCPP_INFO(get_logger(), "Exploration goal cancel requested.");
  goal_changed_ = true;
  cancelCurrentGoal();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GraphExplorer::handleAccepted(const std::shared_ptr<ExploreHandle> handle)
{
  // If a goal is already running, interrupt it — the exploration thread will
  // detect goal_changed_, abort the old handle, then pick up the new one.
  {
    std::lock_guard<std::mutex> lock(explore_handle_mutex_);
    active_explore_handle_ = handle;
  }
  goal_changed_ = true;
  cancelCurrentGoal();

  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    final_goal_ = handle->get_goal()->target_pose;
    has_goal_   = true;
  }
  {
    std::lock_guard<std::mutex> lock(visited_mutex_);
    visited_waypoints_.clear();
  }
  {
    std::lock_guard<std::mutex> lock(explored_mutex_);
    explored_frontiers_.clear();
  }
  goal_cv_.notify_one();
}

void GraphExplorer::onGraphCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Layout: x(f32,0) y(f32,4) z(f32,8) penalty(f32,12) neighbor_count(f32,16), step=20
  if (msg->point_step < 20 || msg->width == 0) return;

  std::vector<GraphNode> nodes;
  nodes.reserve(msg->width);
  const uint8_t* base = msg->data.data();
  for (uint32_t i = 0; i < msg->width; ++i) {
    const float* p = reinterpret_cast<const float*>(base + i * msg->point_step);
    nodes.push_back({p[0], p[1], p[2], p[3], p[4]});
  }
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    graph_nodes_ = std::move(nodes);
  }
  RCLCPP_DEBUG(get_logger(), "Graph cloud received: %u walkable nodes.", msg->width);
}

void GraphExplorer::onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
  auto* tree_raw = octomap_msgs::msgToMap(*msg);
  if (!tree_raw) return;
  auto* ocTree = dynamic_cast<octomap::OcTree*>(tree_raw);
  if (!ocTree) { delete tree_raw; return; }

  std::lock_guard<std::mutex> lock(octree_mutex_);
  octree_.reset(ocTree);
}

// =============================================================================
// Exploration thread
// =============================================================================

void GraphExplorer::explorationLoop()
{
  while (!shutdown_) {
    std::shared_ptr<ExploreHandle> handle;
    geometry_msgs::msg::PoseStamped current_goal;
    {
      std::unique_lock<std::mutex> lock(goal_mutex_);
      goal_cv_.wait(lock, [this] { return has_goal_ || shutdown_.load(); });
      if (shutdown_) break;
      current_goal  = final_goal_;
      goal_changed_ = false;
    }
    {
      std::lock_guard<std::mutex> lock(explore_handle_mutex_);
      handle = active_explore_handle_;
    }
    if (handle) {
      explore(handle);
    }
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (!goal_changed_) has_goal_ = false;
    }
  }
}

void GraphExplorer::explore(const std::shared_ptr<ExploreHandle>& handle)
{
  const auto& final_goal = handle->get_goal()->target_pose;

  const auto t_start = std::chrono::steady_clock::now();
  auto elapsed_s = [&]() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
  };

  // Helper: compute result fields from current robot pose
  auto make_result = [&](float rx, float ry) {
    auto result = std::make_shared<ExploreToGoal::Result>();
    const float dx = static_cast<float>(final_goal.pose.position.x) - rx;
    const float dy = static_cast<float>(final_goal.pose.position.y) - ry;
    result->dist_to_goal  = std::sqrt(dx*dx + dy*dy);
    result->angle_to_goal = std::atan2(dy, dx);
    return result;
  };

  // Helper: publish distance feedback
  auto publish_feedback = [&](float rx, float ry) {
    auto fb = std::make_shared<ExploreToGoal::Feedback>();
    const float dx = static_cast<float>(final_goal.pose.position.x) - rx;
    const float dy = static_cast<float>(final_goal.pose.position.y) - ry;
    fb->distance_to_goal = std::sqrt(dx*dx + dy*dy);
    handle->publish_feedback(fb);
  };

  RCLCPP_INFO(get_logger(), "Starting exploration toward (%.2f, %.2f, %.2f)",
    final_goal.pose.position.x, final_goal.pose.position.y, final_goal.pose.position.z);

  const float gx = static_cast<float>(final_goal.pose.position.x);
  const float gy = static_cast<float>(final_goal.pose.position.y);

  int empty_cycles = 0;

  // Stuck detection state
  float stuck_ref_x = std::numeric_limits<float>::quiet_NaN();
  float stuck_ref_y = std::numeric_limits<float>::quiet_NaN();
  auto  stuck_ref_time = std::chrono::steady_clock::now();

  // last known robot pose for result computation even if TF fails at exit
  float last_rx = gx, last_ry = gy;

  while (!shutdown_ && !goal_changed_) {
    // Check if the action client has requested cancellation
    if (handle->is_canceling()) {
      RCLCPP_INFO(get_logger(), "Exploration cancelled by client.");
      handle->canceled(make_result(last_rx, last_ry));
      return;
    }

    // 1. Robot pose
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!getRobotPose(robot_pose)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
      continue;
    }
    const float rx = static_cast<float>(robot_pose.pose.position.x);
    const float ry = static_cast<float>(robot_pose.pose.position.y);
    const float rz = static_cast<float>(robot_pose.pose.position.z);
    last_rx = rx; last_ry = ry;
    const auto& q  = robot_pose.pose.orientation;
    const float robot_yaw = static_cast<float>(std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z)));

    // 1b. Stuck detection
    if (!std::isnan(stuck_ref_x)) {
      const float sdx = rx - stuck_ref_x, sdy = ry - stuck_ref_y;
      if (sdx*sdx + sdy*sdy > static_cast<float>(stuck_move_threshold_ * stuck_move_threshold_)) {
        stuck_ref_x = rx; stuck_ref_y = ry;
        stuck_ref_time = std::chrono::steady_clock::now();
      } else {
        const double stuck_elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - stuck_ref_time).count();
        if (stuck_elapsed > stuck_timeout_) {
          RCLCPP_WARN(get_logger(),
            "Robot stuck for %.1f s at (%.2f, %.2f) — cancelling goal and reversing %.2f m.",
            stuck_elapsed, rx, ry, stuck_backup_dist_);
          cancelCurrentGoal();
          driveBackward(stuck_backup_dist_);
          stuck_ref_x = rx; stuck_ref_y = ry;
          stuck_ref_time = std::chrono::steady_clock::now();
          {
            std::lock_guard<std::mutex> lock(visited_mutex_);
            visited_waypoints_.clear();
          }
          continue;
        }
      }
    } else {
      stuck_ref_x = rx; stuck_ref_y = ry;
      stuck_ref_time = std::chrono::steady_clock::now();
    }

    // 1c. Feedback
    publish_feedback(rx, ry);

    // 2. Goal reached or close enough to send directly?
    const float dx = rx - gx, dy = ry - gy;
    const float dist_to_goal = std::sqrt(dx*dx + dy*dy);

    if (dist_to_goal < static_cast<float>(goal_tolerance_)) {
      RCLCPP_INFO(get_logger(),
        "Final goal reached (dist_xy=%.2fm). Total exploration time: %.1f s",
        dist_to_goal, elapsed_s());
      handle->succeed(make_result(rx, ry));
      return;
    }

    if (dist_to_goal < 0.5f) {
      RCLCPP_INFO(get_logger(),
        "Within 0.5m of goal — sending final goal directly (elapsed: %.1f s).", elapsed_s());
      sendMoveBaseGoal(final_goal);
      RCLCPP_INFO(get_logger(), "Exploration complete. Total time: %.1f s", elapsed_s());
      handle->succeed(make_result(rx, ry));
      return;
    }

    // 3. Snapshot graph and octree
    std::shared_ptr<octomap::OcTree> octree_snap;
    {
      std::lock_guard<std::mutex> lock(octree_mutex_);
      octree_snap = octree_;
    }

    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "Graph has %zu walkable nodes total.", graph_nodes_.size());
      if (graph_nodes_.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "No graph nodes received yet — is /move_base_flex/graph_cloud being published?");
      }
    }
    if (!octree_snap) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "No octomap received yet — waiting for '%s'.", octomap_topic_.c_str());
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      continue;
    }

    // 3b. If the goal is already within the explored graph, send it directly.
    {
      const float goal_reach_r2 = 0.5f * 0.5f;
      bool goal_in_graph = false;
      {
        std::lock_guard<std::mutex> lock(graph_mutex_);
        for (const auto& n : graph_nodes_) {
          const float ddx = n.x - gx, ddy = n.y - gy;
          if (ddx*ddx + ddy*ddy < goal_reach_r2) { goal_in_graph = true; break; }
        }
      }
      if (goal_in_graph) {
        RCLCPP_INFO(get_logger(),
          "Goal is inside the explored graph — sending final goal directly (elapsed: %.1f s).",
          elapsed_s());
        sendMoveBaseGoal(final_goal);
        RCLCPP_INFO(get_logger(), "Exploration complete. Total time: %.1f s", elapsed_s());
        handle->succeed(make_result(rx, ry));
        return;
      }
    }

    // 4. Find frontier candidates
    auto candidates = findCandidates(rx, ry, rz, robot_yaw, gx, gy, octree_snap);
    RCLCPP_INFO(get_logger(),
      "Candidates within %.1fm: %zu (robot=%.2f,%.2f goal=%.2f,%.2f)",
      frontier_radius_, candidates.size(), rx, ry, gx, gy);

    // 5. No candidates → wait or try direct goal
    if (candidates.empty()) {
      ++empty_cycles;
      RCLCPP_WARN(get_logger(),
        "No frontier candidates in radius %.1fm (cycle %d).", frontier_radius_, empty_cycles);
      if (empty_cycles >= 3) {
        RCLCPP_WARN(get_logger(),
          "Sending final goal directly as last resort (elapsed: %.1f s).", elapsed_s());
        sendMoveBaseGoal(final_goal);
        RCLCPP_INFO(get_logger(), "Exploration complete. Total time: %.1f s", elapsed_s());
        handle->succeed(make_result(rx, ry));
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      continue;
    }
    empty_cycles = 0;

    // 6. Select best candidate
    const auto best = selectBest(candidates, rx, ry, robot_yaw, gx, gy);
    RCLCPP_INFO(get_logger(),
      "Best waypoint: (%.2f,%.2f,%.2f) clearance=%.2fm node_pen=%.2f neighbors=%.0f",
      best.x, best.y, best.z, best.clearance, best.penalty, best.neighbor_count);

    // 7. Navigate to waypoint. Orient along the direction of travel so the
    // robot arrives facing onward — the next cycle's front cone then continues
    // the same path instead of forcing a turn-around.
    const auto target  = makePoseToward(best.x, best.y, best.z,
                                        2.f * best.x - rx, 2.f * best.y - ry);
    const bool reached = sendMoveBaseGoal(target);
    {
      std::lock_guard<std::mutex> lock(visited_mutex_);
      visited_waypoints_.push_back({best.x, best.y, best.z});
    }
    {
      std::lock_guard<std::mutex> lock(explored_mutex_);
      explored_frontiers_.push_back({best.x, best.y, best.z});
      RCLCPP_INFO(get_logger(), "Stored frontier (%.2f,%.2f) — %zu total explored areas.",
        best.x, best.y, explored_frontiers_.size());
    }
    if (reached) {
      RCLCPP_INFO(get_logger(), "Waypoint reached — waiting 3 s for map to update.");
      for (int i = 0; i < 30 && !shutdown_ && !goal_changed_; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    } else if (!goal_changed_ && !shutdown_) {
      RCLCPP_WARN(get_logger(), "Failed to reach waypoint, blacklisting and retrying.");
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
  }

  // Exited loop because goal_changed_ or shutdown_ — abort the handle if it's still ours
  if (!shutdown_) {
    std::shared_ptr<ExploreHandle> current_handle;
    {
      std::lock_guard<std::mutex> lock(explore_handle_mutex_);
      current_handle = active_explore_handle_;
    }
    // Only abort if this is still the active handle (not replaced by a newer goal)
    if (current_handle.get() == handle.get() &&
        handle->is_active() && !handle->is_canceling()) {
      handle->abort(make_result(last_rx, last_ry));
    }
  }
}

// =============================================================================
// TF
// =============================================================================

bool GraphExplorer::getRobotPose(geometry_msgs::msg::PoseStamped& pose)
{
  try {
    auto tf = tf_buffer_.lookupTransform(map_frame_, robot_frame_, tf2::TimePointZero);
    pose.header.frame_id  = map_frame_;
    pose.header.stamp     = tf.header.stamp;
    pose.pose.position.x  = tf.transform.translation.x;
    pose.pose.position.y  = tf.transform.translation.y;
    pose.pose.position.z  = tf.transform.translation.z;
    pose.pose.orientation = tf.transform.rotation;
    return true;
  } catch (const tf2::TransformException& e) {
    RCLCPP_WARN(get_logger(), "TF lookup failed: %s", e.what());
    return false;
  }
}

// =============================================================================
// Robot-position clearance — 9 robot-relative directions × 3 origins
//
// Directions (relative to robot yaw):
//   ±22.5° (front), ±45°, ±90° (sides), ±135°, 180° (back)
//
// Origins: base_link (rz), rz-0.2 m, rz-0.6 m
//
// Direction score = 0.6 × no_hit_fraction + 0.3 × norm_goal_align + 0.1 × norm_avg_clear
//   no_hit_fraction = origins where ray reached kRobotRayMaxDist / kNumOrigins ∈ [0, 1]
//   norm_goal_align = (dot(dir, robot→goal) + 1) / 2                           ∈ [0, 1]
//   norm_avg_clear  = avg_hit_dist / kRobotRayMaxDist                           ∈ [0, 1]
//
// Directions where all beams travel unobstructed score highest; goal direction
// breaks ties.  All 15 rays published as a pink LINE_LIST on ~/exploration_rays.
// =============================================================================

std::pair<float,float> GraphExplorer::computeRobotClearanceDir(
    float rx, float ry, float rz,
    float robot_yaw,
    float goal_x, float goal_y,
    const octomap::OcTree& tree)
{
  constexpr float k22  = static_cast<float>(M_PI / 8.0);   // 22.5°
  constexpr float k45  = static_cast<float>(M_PI / 4.0);   // 45°
  constexpr float k90  = static_cast<float>(M_PI / 2.0);   // 90°
  constexpr float k135 = static_cast<float>(3.0 * M_PI / 4.0);  // 135°
  constexpr float k180 = static_cast<float>(M_PI);          // 180°
  const float ray_angles[] = {
    robot_yaw + k22,   // front-left
    robot_yaw - k22,   // front-right
    robot_yaw + k45,   // left-front diagonal
    robot_yaw - k45,   // right-front diagonal
    robot_yaw + k90,   // left
    robot_yaw - k90,   // right
    robot_yaw + k135,  // left-rear diagonal
    robot_yaw - k135,  // right-rear diagonal
    robot_yaw + k180,  // back
  };
  constexpr int kNumDirs = 9;

  // 3 origins: base_link, 0.2 m below, 0.6 m below
  const octomap::point3d origins[] = {
    {rx, ry, rz       },
    {rx, ry, rz - 0.2f},
    {rx, ry, rz - 0.6f},
  };
  constexpr int kNumOrigins = 3;

  const float gdx = goal_x - rx, gdy = goal_y - ry;
  const float gd  = std::sqrt(gdx*gdx + gdy*gdy);
  const float gux = (gd > 0.01f) ? gdx/gd : std::cos(robot_yaw);
  const float guy = (gd > 0.01f) ? gdy/gd : std::sin(robot_yaw);

  float best_score = -std::numeric_limits<float>::infinity();
  float best_dx    = std::cos(robot_yaw);
  float best_dy    = std::sin(robot_yaw);

  visualization_msgs::msg::MarkerArray ma;
  auto& m           = ma.markers.emplace_back();
  m.header.frame_id = map_frame_;
  m.header.stamp    = now();
  m.ns              = "exploration_rays";
  m.id              = 0;
  m.type            = visualization_msgs::msg::Marker::LINE_LIST;
  m.action          = visualization_msgs::msg::Marker::ADD;
  m.scale.x         = 0.04;
  m.color.r         = 1.0f;
  m.color.g         = 0.41f;
  m.color.b         = 0.71f;
  m.color.a         = 1.0f;
  m.pose.orientation.w = 1.0;
  m.lifetime        = rclcpp::Duration::from_seconds(0.5);

  for (int i = 0; i < kNumDirs; ++i) {
    const float dx = std::cos(ray_angles[i]);
    const float dy = std::sin(ray_angles[i]);
    const octomap::point3d dir(dx, dy, 0.f);

    int   no_hit_count     = 0;
    float total_clearance  = 0.f;

    for (int o = 0; o < kNumOrigins; ++o) {
      octomap::point3d end;
      const bool hit = tree.castRay(origins[o], dir, end, /*ignoreUnknown=*/true, kRobotRayMaxDist);
      const float c = hit ? static_cast<float>((end - origins[o]).norm()) : kRobotRayMaxDist;
      if (!hit) ++no_hit_count;
      total_clearance += c;

      geometry_msgs::msg::Point p0, p1;
      p0.x = origins[o].x(); p0.y = origins[o].y(); p0.z = origins[o].z();
      if (hit) {
        p1.x = end.x(); p1.y = end.y(); p1.z = end.z();
      } else {
        p1.x = origins[o].x() + dx * kRobotRayMaxDist;
        p1.y = origins[o].y() + dy * kRobotRayMaxDist;
        p1.z = origins[o].z();
      }
      m.points.push_back(p0);
      m.points.push_back(p1);
    }

    const float no_hit_frac = static_cast<float>(no_hit_count) / kNumOrigins;
    const float norm_clear  = (total_clearance / kNumOrigins) / kRobotRayMaxDist;
    const float goal_align  = dx * gux + dy * guy;
    const float norm_goal   = (goal_align + 1.0f) * 0.5f;
    // Primary: how many beams are unobstructed; secondary: goal direction; tertiary: avg dist
    const float score = 0.6f * no_hit_frac + 0.3f * norm_goal + 0.1f * norm_clear;

    if (score > best_score) {
      best_score = score;
      best_dx = dx; best_dy = dy;
    }
  }

  ray_marker_pub_->publish(ma);
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    "Best dir: (%.2f, %.2f)  score=%.2f", best_dx, best_dy, best_score);
  return {best_dx, best_dy};
}

// =============================================================================
// Clearance via raycasting
//
// 4 horizontal directions (±x, ±y) × 3 heights (z+0.5, z+0.7, z+0.9 m).
// Returns the MINIMUM per-direction clearance (each direction averaged over
// the 3 heights) — one nearby wall in any direction tanks the score.
// =============================================================================

float GraphExplorer::computeClearance(const octomap::OcTree& tree,
                                       float x, float y, float z) const
{
  static const octomap::point3d kDirs[4] = {
    { 1.f, 0.f, 0.f},
    {-1.f, 0.f, 0.f},
    { 0.f, 1.f, 0.f},
    { 0.f,-1.f, 0.f},
  };

  float min_dir_clearance = kRayMaxDist;

  for (const auto& dir : kDirs) {
    float dir_sum = 0.0f;
    for (float dz : kRayHeights) {
      const octomap::point3d origin(x, y, z + dz);
      octomap::point3d end;
      const bool hit = tree.castRay(origin, dir, end,
                                    /*ignoreUnknown=*/true,
                                    /*maxRange=*/kRayMaxDist);
      dir_sum += hit ? static_cast<float>((end - origin).norm()) : kRayMaxDist;
    }
    const float dir_clearance = dir_sum / static_cast<float>(sizeof(kRayHeights) / sizeof(kRayHeights[0]));
    if (dir_clearance < min_dir_clearance) min_dir_clearance = dir_clearance;
  }

  return min_dir_clearance;
}

// =============================================================================
// Candidate search
//
// 1. Penalty threshold: compute max penalty across the full graph; nodes above
//    candidate_penalty_ratio × max_penalty are "red" (wall-adjacent) and excluded.
//
// 2. Robot raycasts → clearance direction (5 dirs × 3 origins, no-hit scoring).
//
// 3. Global frontier direction: scan ALL graph nodes for low-neighbor-count,
//    low-penalty nodes (edge of the explored map).  Average the directions from
//    the robot to the best-scoring frontier nodes → "frontier bearing".
//    Blend: final_dir = normalize(0.6 × clearance_dir + 0.4 × frontier_dir).
//
// 4. Collect local graph nodes (within frontier_radius_) that face final_dir
//    AND pass the penalty filter.  Fallback: full radius if cone is empty.
//
// 5. Per-candidate clearance raycasts (12 rays) for scoring in selectBest().
// =============================================================================

std::vector<GraphExplorer::Candidate> GraphExplorer::findCandidates(
    float robot_x, float robot_y, float robot_z,
    float robot_yaw,
    float goal_x, float goal_y,
    const std::shared_ptr<octomap::OcTree>& tree)
{
  const float local_r2 = static_cast<float>(frontier_radius_ * frontier_radius_);

  // --- 1. Penalty threshold + wall-top Z threshold (one pass over graph) ------
  float max_pen     = 0.f;
  float max_graph_z = -std::numeric_limits<float>::max();
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    for (const auto& n : graph_nodes_) {
      max_pen     = std::max(max_pen,     n.penalty);
      max_graph_z = std::max(max_graph_z, n.z);
    }
  }
  const float pen_thresh  = max_pen * static_cast<float>(candidate_penalty_ratio_);
  // Nodes within 0.2 m of the highest graph Z are almost certainly wall tops
  const float wall_z_max  = max_graph_z - 0.2f;

  // --- 2. Robot-position raycasts → clearance direction ----------------------
  float dir_x, dir_y;
  if (tree) {
    std::tie(dir_x, dir_y) = computeRobotClearanceDir(
      robot_x, robot_y, robot_z, robot_yaw, goal_x, goal_y, *tree);
  } else {
    const float gdx = goal_x - robot_x, gdy = goal_y - robot_y;
    const float gd  = std::sqrt(gdx*gdx + gdy*gdy);
    dir_x = (gd > 0.01f) ? gdx/gd : 1.f;
    dir_y = (gd > 0.01f) ? gdy/gd : 0.f;
  }

  // --- 3. Global frontier bearing (full map, edge-of-graph nodes) -------------
  {
    // Unit vector robot→goal for scoring frontier nodes
    const float gdx = goal_x - robot_x, gdy = goal_y - robot_y;
    const float gd  = std::sqrt(gdx*gdx + gdy*gdy);
    const float gux = (gd > 0.01f) ? gdx/gd : dir_x;
    const float guy = (gd > 0.01f) ? gdy/gd : dir_y;

    struct FNode { float x, y, score; };
    std::vector<FNode> fnodes;
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      for (const auto& n : graph_nodes_) {
        if (n.penalty > pen_thresh)                                           continue;
        if (n.neighbor_count > static_cast<float>(frontier_max_neighbors_))  continue;
        const float fdx = n.x - robot_x, fdy = n.y - robot_y;
        const float fd  = std::sqrt(fdx*fdx + fdy*fdy);
        if (fd < 0.3f) continue;
        // Score: goal-aligned frontier nodes score highest
        const float align = (fdx/fd)*gux + (fdy/fd)*guy;
        fnodes.push_back({n.x, n.y, align});
      }
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "Frontier nodes (low-neighbor, low-penalty): %zu", fnodes.size());

    if (!fnodes.empty()) {
      const std::size_t keep = std::min(fnodes.size(), std::size_t(20));
      std::partial_sort(fnodes.begin(), fnodes.begin() + keep, fnodes.end(),
        [](const FNode& a, const FNode& b){ return a.score > b.score; });

      float sum_dx = 0.f, sum_dy = 0.f;
      for (std::size_t i = 0; i < keep; ++i) {
        const float fdx = fnodes[i].x - robot_x, fdy = fnodes[i].y - robot_y;
        const float fd  = std::sqrt(fdx*fdx + fdy*fdy);
        if (fd > 0.01f) { sum_dx += fdx/fd; sum_dy += fdy/fd; }
      }
      const float sd = std::sqrt(sum_dx*sum_dx + sum_dy*sum_dy);
      if (sd > 0.01f) {
        const float fx = sum_dx/sd, fy = sum_dy/sd;
        // Blend: 60% clearance (immediate safety), 40% frontier (map edges)
        const float bx = 0.6f*dir_x + 0.4f*fx;
        const float by = 0.6f*dir_y + 0.4f*fy;
        const float bd = std::sqrt(bx*bx + by*by);
        if (bd > 0.01f) { dir_x = bx/bd; dir_y = by/bd; }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
          "Blended dir: (%.2f, %.2f) from clearance + %zu frontier nodes",
          dir_x, dir_y, keep);
      }
    }
  }

  // --- 4. Collect local candidates aligned with final direction ---------------
  std::vector<Candidate> out;
  auto collectNodes = [&](bool filter_by_bearing) {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    for (const auto& n : graph_nodes_) {
      if (n.penalty > pen_thresh) continue;  // skip red/wall-adjacent nodes
      if (n.z       > wall_z_max) continue;  // skip probable wall-top nodes
      const float dx = n.x - robot_x, dy = n.y - robot_y, dz = n.z - robot_z;
      if (dx*dx + dy*dy > local_r2) continue;
      if (dz >  0.3f)  continue;  // never climb more than 0.3 m above base_link
      if (dz < -1.0f)  continue;  // ignore nodes way below
      if (filter_by_bearing) {
        const float nd = std::sqrt(dx*dx + dy*dy);
        if (nd > 0.01f && (dx/nd)*dir_x + (dy/nd)*dir_y < 0.05f) continue;
      }
      out.push_back({n.x, n.y, n.z, n.penalty, n.neighbor_count, 0.0f});
    }
  };

  collectNodes(true);
  if (out.empty()) {
    RCLCPP_WARN(get_logger(), "No nodes in open cone — falling back to full radius.");
    collectNodes(false);
  }

  // --- 5. Per-candidate clearance raycasts + wall-top rejection ---------------
  if (tree) {
    std::vector<Candidate> filtered;
    filtered.reserve(out.size());
    for (auto& c : out) {
      c.clearance = computeClearance(*tree, c.x, c.y, c.z);
      // If every horizontal ray reaches max distance the node is likely on top
      // of a wall or cliff with open air on all sides — reject it.
      if (c.clearance >= kRayMaxDist - 0.05f) {
        RCLCPP_DEBUG(get_logger(),
          "Dropping wall-top candidate (%.2f,%.2f,%.2f) clearance=%.2f",
          c.x, c.y, c.z, c.clearance);
        continue;
      }
      filtered.push_back(c);
    }
    if (!filtered.empty()) out = std::move(filtered);
  }

  return out;
}

// =============================================================================
// Candidate scoring
//
// Candidates are grouped into expanding heading cones around the robot's
// current yaw (±30° → ±60° → ±90° → ±135° → 180°).  The narrowest non-empty
// cone wins outright: a frontier in front of the robot always beats one
// behind it, so the robot keeps following its own path forward and only
// turns around when nothing ahead is left to explore.
//
// Within the winning cone:
// Primary: clearance (minimum directional ray distance) — open space wins.
// Secondary: alignment toward goal (weak directional bias allows backtracking).
// Tertiary: distance toward outer search radius, low wall penalty.
// =============================================================================

GraphExplorer::Candidate GraphExplorer::selectBest(
    const std::vector<Candidate>& candidates,
    float robot_x, float robot_y, float robot_yaw,
    float goal_x,  float goal_y)
{
  float gdx   = goal_x - robot_x;
  float gdy   = goal_y - robot_y;
  float gdist = std::sqrt(gdx*gdx + gdy*gdy);
  if (gdist > 0.01f) { gdx /= gdist; gdy /= gdist; }

  const float min_dist  = static_cast<float>(min_waypoint_dist_);
  const float vis_r2    = static_cast<float>(visited_radius_ * visited_radius_);
  const float exp_r2    = static_cast<float>(explored_frontier_radius_ * explored_frontier_radius_);
  const float fr        = static_cast<float>(frontier_radius_);

  std::vector<VisitedWaypoint> visited_snap;
  {
    std::lock_guard<std::mutex> lock(visited_mutex_);
    visited_snap = visited_waypoints_;
  }
  std::vector<VisitedWaypoint> explored_snap;
  {
    std::lock_guard<std::mutex> lock(explored_mutex_);
    explored_snap = explored_frontiers_;
  }

  // Normalization ranges
  float max_clearance = 0.0f;
  float max_pen       = 0.0f;
  for (const auto& c : candidates) {
    max_clearance = std::max(max_clearance, c.clearance);
    max_pen       = std::max(max_pen,       c.penalty);
  }
  if (max_clearance < 1e-4f) max_clearance = kRayMaxDist;
  if (max_pen       < 1e-6f) max_pen       = 1.0f;

  // Hard filters + per-candidate score and heading deviation from robot yaw
  struct Scored { const Candidate* c; float heading_dev; float score; };
  std::vector<Scored> valid;
  valid.reserve(candidates.size());

  for (const auto& c : candidates) {
    const float fdx   = c.x - robot_x;
    const float fdy   = c.y - robot_y;
    const float fdist = std::sqrt(fdx*fdx + fdy*fdy);

    if (fdist < min_dist) continue;

    // Short-range: don't immediately revisit a recent waypoint
    bool visited = false;
    for (const auto& v : visited_snap) {
      const float vdx = c.x - v.x, vdy = c.y - v.y;
      if (vdx*vdx + vdy*vdy < vis_r2) { visited = true; break; }
    }
    if (visited) continue;

    // Long-range: don't re-explore an area already covered this run
    bool already_explored = false;
    for (const auto& e : explored_snap) {
      const float edx = c.x - e.x, edy = c.y - e.y;
      if (edx*edx + edy*edy < exp_r2) { already_explored = true; break; }
    }
    if (already_explored) continue;

    const float alignment  = (fdx/fdist)*gdx + (fdy/fdist)*gdy;
    const float norm_clear = c.clearance / max_clearance;
    const float norm_pen   = c.penalty   / max_pen;
    const float dist_bonus = fdist / fr;

    // Clearance dominates; alignment is a weak tiebreaker so the robot can
    // backtrack to unexplored pockets rather than always heading toward goal.
    const float score = 0.65f * norm_clear
                      + 0.20f * alignment
                      + 0.15f * dist_bonus
                      - static_cast<float>(penalty_weight_) * norm_pen;

    const float heading_dev = std::abs(static_cast<float>(
      std::remainder(std::atan2(fdy, fdx) - robot_yaw, 2.0 * M_PI)));

    valid.push_back({&c, heading_dev, score});
  }

  // Expanding heading cones: the narrowest cone containing any valid candidate
  // wins, so frontiers in front of the robot always beat ones behind it.
  static constexpr float kConeHalfAngles[] = {
    static_cast<float>(M_PI / 6.0),        //  ±30°
    static_cast<float>(M_PI / 3.0),        //  ±60°
    static_cast<float>(M_PI / 2.0),        //  ±90°
    static_cast<float>(3.0 * M_PI / 4.0),  // ±135°
    static_cast<float>(M_PI),              //  180° (full circle)
  };

  const Scored* best = nullptr;
  for (float cone : kConeHalfAngles) {
    for (const auto& s : valid) {
      if (s.heading_dev > cone) continue;
      if (!best || s.score > best->score) best = &s;
    }
    if (best) {
      RCLCPP_INFO(get_logger(),
        "Selected frontier within ±%.0f° front cone (heading_dev=%.0f°, %zu valid candidates).",
        cone * 180.0 / M_PI, best->heading_dev * 180.0 / M_PI, valid.size());
      break;
    }
  }

  if (!best) {
    RCLCPP_WARN(get_logger(),
      "All candidates filtered (too close or visited). Clearing visited list.");
    std::lock_guard<std::mutex> lock(visited_mutex_);
    visited_waypoints_.clear();
    return candidates.front();
  }
  return *best->c;
}

// =============================================================================
// Navigation
// =============================================================================

bool GraphExplorer::sendMoveBaseGoal(const geometry_msgs::msg::PoseStamped& target)
{
  if (!move_base_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_logger(), "MoveBase action server not available.");
    return false;
  }

  auto promise = std::make_shared<std::promise<bool>>();
  auto future  = promise->get_future();

  auto goal        = MoveBase::Goal();
  goal.target_pose = target;
  goal.planner     = "octo_planner";
  goal.controller  = "octo_controller";

  auto opts = rclcpp_action::Client<MoveBase>::SendGoalOptions();

  opts.goal_response_callback = [this, promise](MBFGoalHandle::SharedPtr mbf_handle) {
    if (!mbf_handle) {
      RCLCPP_ERROR(get_logger(), "MoveBase goal rejected.");
      promise->set_value(false);
      return;
    }
    std::lock_guard<std::mutex> lock(goal_handle_mutex_);
    active_goal_handle_ = mbf_handle;
    RCLCPP_INFO(get_logger(), "MoveBase goal accepted, executing...");
  };

  opts.result_callback = [this, promise](
      const rclcpp_action::ClientGoalHandle<MoveBase>::WrappedResult& result) {
    const bool ok = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
    RCLCPP_INFO(get_logger(), "MoveBase result: %s (outcome=%u)",
      ok ? "SUCCEEDED" : "FAILED/CANCELED", result.result->outcome);
    promise->set_value(ok);
    std::lock_guard<std::mutex> lock(goal_handle_mutex_);
    active_goal_handle_ = nullptr;
  };

  move_base_client_->async_send_goal(goal, opts);

  while (future.wait_for(std::chrono::milliseconds(100)) == std::future_status::timeout) {
    if (shutdown_ || goal_changed_) {
      cancelCurrentGoal();
      future.wait_for(std::chrono::seconds(3));
      return false;
    }
  }
  return future.get();
}

void GraphExplorer::cancelCurrentGoal()
{
  std::shared_ptr<MBFGoalHandle> handle;
  {
    std::lock_guard<std::mutex> lock(goal_handle_mutex_);
    handle = active_goal_handle_;
  }
  if (handle) move_base_client_->async_cancel_goal(handle);
}

// =============================================================================

geometry_msgs::msg::PoseStamped GraphExplorer::makePoseToward(
    float px, float py, float pz, float tx, float ty)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = map_frame_;
  p.header.stamp    = now();
  p.pose.position.x = static_cast<double>(px);
  p.pose.position.y = static_cast<double>(py);
  p.pose.position.z = static_cast<double>(pz);
  const double yaw  = std::atan2(ty - py, tx - px);
  p.pose.orientation.z = std::sin(yaw / 2.0);
  p.pose.orientation.w = std::cos(yaw / 2.0);
  return p;
}

// =============================================================================
// Stuck recovery: publish negative linear-x velocity for dist_m / speed_mps seconds
// =============================================================================

void GraphExplorer::driveBackward(double dist_m, double speed_mps)
{
  const double duration_s = dist_m / speed_mps;
  const auto end_time = std::chrono::steady_clock::now()
    + std::chrono::duration<double>(duration_s);

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = -speed_mps;

  RCLCPP_INFO(get_logger(), "Reversing at %.2f m/s for %.2f s (%.2f m).",
    speed_mps, duration_s, dist_m);

  while (std::chrono::steady_clock::now() < end_time && !shutdown_ && !goal_changed_) {
    cmd_vel_pub_->publish(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // Stop
  geometry_msgs::msg::Twist stop;
  cmd_vel_pub_->publish(stop);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

}  // namespace graph_exploration
