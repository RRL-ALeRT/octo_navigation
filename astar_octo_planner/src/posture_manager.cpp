// posture_manager.cpp - ROS 2 node that lowers Spot's body height when approaching a SquatPath
// Author: 
//
// This node subscribes to a Path published by astar_octo_planner that marks the segment
// where the robot must crouch ("squat") under an overhead obstacle.  When the robot is
// within a configurable distance of the first waypoint in that SquatPath it sends a
// SetBodyHeight request via the Spot ROS driver to lower the body.  Once it has cleared
// the last SquatPath waypoint (plus a small hysteresis distance) it commands Spot to
// stand up again.
//
// Parameters (all dynamic‑reconfigure/runtime‑settable via ROS 2 params):
//   squat_path_topic (string)   – topic name of the SquatPath (nav_msgs/Path)
//   odom_topic        (string)   – topic publishing nav_msgs/Odometry (or tf lookup)
//   height_service    (string)   – service to call for changing body height
//   approach_dist     (double)   – start crouching when robot is closer than this (m)
//   hysteresis_dist   (double)   – stand up again once this far beyond last waypoint (m)
//   squat_height      (double)   – relative height (m) to command when crouching
//   stand_height      (double)   – relative height (m) to command when standing
//
// Build: add this file to your CMakeLists.txt executable and link against rclcpp and
// the Spot message package that provides bosdyn_msgs/srv/SetBodyHeight (or adjust
// the include + service type below to match your driver).

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"
#include <mbf_msgs/action/exe_path.hpp>

// Replace with the service header for Spot driver.
// For the Boston Dynamics driver is bosdyn_msgs/srv/SetBodyHeight.(?)
#include "webots_spot_msgs/srv/spot_height.hpp"

using namespace std::chrono_literals;

class PostureManager : public rclcpp::Node
{
public:
  using ExePath = mbf_msgs::action::ExePath;
  using ResultMsg = mbf_msgs::action::ExePath::Result;

  explicit PostureManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("posture_manager", options)
  {
    /* Declare and read parameters */
    squat_path_topic_ = declare_parameter<std::string>("squat_path_topic", "/move_base_flex/Squatpath");
    odom_topic_       = declare_parameter<std::string>("odom_topic", "/Spot/odometry");
    height_service_   = declare_parameter<std::string>("height_service", "/Spot/set_height");
    approach_dist_    = declare_parameter<double>("approach_dist", 0.6);
    hysteresis_dist_  = declare_parameter<double>("hysteresis_dist", 0.4);
    squat_height_     = declare_parameter<double>("squat_height", -0.20);
    stand_height_     = declare_parameter<double>("stand_height", 0.0);

    /* Create subscriptions */
    // ExePath status & result
    status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
      "/move_base_flex/exe_path/_action/status", rclcpp::SystemDefaultsQoS(),
      std::bind(&PostureManager::onExeStatus, this, std::placeholders::_1));
    
    result_sub_ = create_subscription<ResultMsg>(
      "/move_base_flex/exe_path/_action/result", rclcpp::SystemDefaultsQoS(),
      [this](const ResultMsg::SharedPtr msg) { this->onExeResult(*msg); });

    // SquatPath & odom
    squat_sub_ = create_subscription<nav_msgs::msg::Path>(
      squat_path_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&PostureManager::onSquatPath, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PostureManager::onOdom, this, std::placeholders::_1));

    /* Create service client */
    height_cli_ = create_client<webots_spot_msgs::srv::SpotHeight>(height_service_);

    RCLCPP_INFO(get_logger(), "PostureManager ready - waiting for ExePath goal & SquatPath.");
  }

private:
  /* ---------------- Callbacks ---------------- */
  /* ---------- ExePath status / result ---------- */
  void onExeStatus(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
  {
    for (const auto & st : msg->status_list)
    {
      if (st.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
          st.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
      {
        if (!nav_session_active_ || !std::equal(current_goal_uuid_.uuid.begin(), current_goal_uuid_.uuid.end(), st.goal_info.goal_id.uuid.begin()))
        {
          current_goal_uuid_.uuid = st.goal_info.goal_id.uuid;
          nav_session_active_ = true;
          latchCurrentSquatPath();
          RCLCPP_INFO(get_logger(), "ExePath goal %s active - posture controlled.", uuidToStr(current_goal_uuid_).c_str());
        }
        return;  // had active goal
      }
    }
    // if this array don't have ACCEPTED/EXECUTING，but nav_session_active_ still true，wait result return reset
  }
  void onExeResult(const mbf_msgs::action::ExePath::Result & msg)
  {
    // ExePath::Result does not contain goal_id; just reset state if active end
    if (nav_session_active_)
    {
      RCLCPP_INFO(get_logger(), "ExePath goal finished - reset posture.");
      resetState();
    }
  }

  /* ---------- SquatPath ---------- */
  void onSquatPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    // ALWAYS save last_seen_squat_path_ for next StartNav latch
    last_seen_squat_path_ = *msg;

    if (!nav_session_active_)
    {
      // hasn't already StartNav latch or clear
      latchCurrentSquatPath();
    }
    else if (msg->poses.empty())
    {
      // naving but planner send out empty path -> stand
      squat_path_.poses.clear();
      if (is_squatting_) sendHeight(stand_height_);
      is_squatting_ = false;
      RCLCPP_INFO(get_logger(), "Received empty SquatPath during nav - cancel crouch.");
    }
  }

  void latchCurrentSquatPath()
  {
    squat_path_ = last_seen_squat_path_;
    is_squatting_ = false;

    if (squat_path_.poses.empty()) return; // no crouch needed

    first_wp_ = squat_path_.poses.front().pose.position;
    last_wp_  = squat_path_.poses.back().pose.position;
    RCLCPP_INFO(get_logger(), "SquatPath latched (%zu poses) first=(%.2f,%.2f) last=(%.2f,%.2f)",
                squat_path_.poses.size(), first_wp_.x, first_wp_.y, last_wp_.x, last_wp_.y);
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!nav_session_active_ || squat_path_.poses.empty()) 
    {
      return;
    }
    const geometry_msgs::msg::Point & p = msg->pose.pose.position;
    
    double dist_to_first = distance2D(p, first_wp_);
    double dist_to_last  = distance2D(p, last_wp_);
    
    // Begin crouch when approaching the first waypoint.
    if (!is_squatting_ && dist_to_first < approach_dist_)
    {
      RCLCPP_INFO(get_logger(), "dist_to_first = %f ,dist_to_last = %f.", dist_to_first, dist_to_last);
      RCLCPP_INFO(get_logger(), "Approach obstacle - lower body (%.2f m)", squat_height_);
      if (sendHeight(squat_height_))
        is_squatting_ = true;
    }
    else if (is_squatting_ && dist_to_first < (approach_dist_+hysteresis_dist_))
    {
      is_squatting_ = true;
    }
    // Rise back up once we have cleared the last waypoint plus hysteresis.
    else if (is_squatting_ && dist_to_last > approach_dist_)
    {
      RCLCPP_INFO(get_logger(), "dist_to_first = %f ,dist_to_last = %f.", dist_to_first, dist_to_last);
      RCLCPP_INFO(get_logger(), "Leaving obstacle - raising body (%.2f m)", stand_height_);
      if (sendHeight(stand_height_))
        is_squatting_ = false;
    }

  }

  /* ---------------- Helpers ---------------- */
  static double distance2D(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
  {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::hypot(dx, dy);
  }

  bool sendHeight(double height)
  {
    if (!height_cli_->wait_for_service(0s))
    {
      RCLCPP_WARN(get_logger(), "Height service '%s' not available.",
                  height_service_.c_str());
      return false;
    }

    auto req = std::make_shared<webots_spot_msgs::srv::SpotHeight::Request>();
    req->height = height;  // Adjust field name to match your service definition.

    auto future = height_cli_->async_send_request(req);
    // Optionally wait for result or just fire‑and‑forget.
    return true;
  }

  static bool uuidEqual(const unique_identifier_msgs::msg::UUID & a, const unique_identifier_msgs::msg::UUID & b)
  {
    return std::equal(a.uuid.begin(), a.uuid.end(), b.uuid.begin());
  }
  static std::string uuidToStr(const unique_identifier_msgs::msg::UUID & u)
  {
    std::ostringstream ss;
    for (uint8_t c : u.uuid) ss << std::hex << std::setw(2) << std::setfill('0') << int(c);
    return ss.str();
  }

  void resetState()
  {
    nav_session_active_ = false;
    squat_path_.poses.clear();
    is_squatting_ = false;
  }

  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
  rclcpp::Subscription<mbf_msgs::action::ExePath::Result>::SharedPtr result_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr      squat_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr  odom_sub_;
  rclcpp::Client<webots_spot_msgs::srv::SpotHeight>::SharedPtr height_cli_;

  // stored paths
  nav_msgs::msg::Path last_seen_squat_path_;
  nav_msgs::msg::Path squat_path_;
  geometry_msgs::msg::Point first_wp_{};
  geometry_msgs::msg::Point last_wp_{};

  // session state
  unique_identifier_msgs::msg::UUID current_goal_uuid_{};
  bool nav_session_active_{false};
  bool is_squatting_{false};

  // Parameters
  std::string squat_path_topic_;
  std::string odom_topic_;
  std::string height_service_;
  double approach_dist_;
  double hysteresis_dist_;
  double squat_height_;
  double stand_height_;
};

/* ---------------- Main ---------------- */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PostureManager>());
  rclcpp::shutdown();
  return 0;
}
