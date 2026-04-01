#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mbf_msgs/action/get_path.hpp>
#include <mbf_octo_core/octo_planner.h>
#include <nav_msgs/msg/path.hpp>
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>

namespace astar_octo_planner
{

/**
 * @brief Simple grid-based A* planner operating on an octomap.
 *
 * The planner discretises the XY plane at the octree resolution, queries
 * the octree for ground height (downward rayCast) at each candidate cell,
 * and checks vertical clearance for the robot body before accepting a cell
 * as traversable.  A standard A* with an 8-connected grid is run in the
 * map frame.
 */
class AstarOctoPlanner : public mbf_octo_core::OctoPlanner
{
public:
  using Ptr = std::shared_ptr<AstarOctoPlanner>;

  AstarOctoPlanner() = default;
  ~AstarOctoPlanner() override = default;

  // ── mbf_octo_core::OctoPlanner interface ─────────────────────────────────

  bool initialize(const std::string & name,
                  const rclcpp::Node::SharedPtr & node) override;

  uint32_t makePlan(const geometry_msgs::msg::PoseStamped & start,
                    const geometry_msgs::msg::PoseStamped & goal,
                    double tolerance,
                    std::vector<geometry_msgs::msg::PoseStamped> & plan,
                    double & cost,
                    std::string & message) override;

  bool cancel() override;

private:
  // ── Grid helpers ──────────────────────────────────────────────────────────

  using GridCell = std::pair<int, int>;

  struct GridCellHash {
    std::size_t operator()(const GridCell & p) const noexcept
    {
      return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 16);
    }
  };

  /** Euclidean distance heuristic scaled to world units. */
  double heuristic(int x1, int y1, int x2, int y2) const;

  /** World → grid (rounds to nearest cell centre). */
  GridCell worldToGrid(double wx, double wy) const;

  /** Grid → world (cell centre). */
  std::pair<double, double> gridToWorld(int gx, int gy) const;

  // ── Octree queries ────────────────────────────────────────────────────────

  /**
   * @brief Iterate over all occupied leaves inside the robot's axis-aligned
   *        bounding box (AABB) at body height.  Faster than point-sampling
   *        because octomap's BBX iterator prunes entire subtrees outside the box.
   *        Returns false if any real (non-anomaly) occupied voxel is found.
   */
  bool checkRadialClearance(double wx, double wy, double z_floor) const;

  /**
   * @brief Returns true if the occupied voxel at (wx,wy,wz) is an anomaly
   *        (sensor noise / isolated point cloud hit) — i.e. it has fewer than
   *        anomaly_min_neighbors_ occupied face-neighbors in the 6-connected
   *        sense.  Such voxels are ignored by traversability checks.
   */
  bool isAnomalyVoxel(double wx, double wy, double wz) const;

  /**
   * @brief Scan the full-height column at (gx, gy) and return the voxel
   *        centers (from the source octree iterator) of ALL occupied voxels
   *        that pass checkRadialClearance.  Empty vector = cell not traversable.
   *        Returning actual source positions ensures the traversable-cells
   *        visualization never publishes voxels absent from the source octomap.
   */
  std::vector<octomap::point3d> cellFloorZ(int gx, int gy) const;

  /** Convenience wrapper — true iff cellFloorZ returns a non-NaN value. */
  bool isCellTraversable(int gx, int gy) const;

  /**
   * @brief Find the nearest traversable cell to (gx, gy) by expanding search.
   *        Returns the input cell if it's traversable, else searches neighbors
   *        in expanding squares up to max_search_radius cells.
   * @param search_z  Ray origin Z.
   * @return The snapped grid cell, or the input if no traversable cell was found.
   */
  GridCell findNearestTraversableCell(int gx, int gy,
                                      int max_search_radius = 5) const;

  // ── Path reconstruction ───────────────────────────────────────────────────

  std::vector<geometry_msgs::msg::PoseStamped> reconstructPath(
    const std::unordered_map<GridCell, GridCell, GridCellHash> & came_from,
    const std::unordered_map<GridCell, std::vector<octomap::point3d>, GridCellHash> & trav_cache,
    GridCell current,
    const std::string & frame_id) const;

  // ── Octomap subscription ──────────────────────────────────────────────────

  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);

  // ── ROS handles ──────────────────────────────────────────────────────────

  rclcpp::Node::SharedPtr node_;
  std::string name_;
  std::atomic<bool> cancel_planning_{false};

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr traversable_cells_pub_;

  mutable std::mutex octree_mutex_;
  std::shared_ptr<octomap::OcTree> octree_;

  // ── Parameters ────────────────────────────────────────────────────────────

  std::string octomap_topic_{"octomap_binary"};
  std::string path_topic_{"~/path"};
  std::string map_frame_{"map"};
  double resolution_{0.1};   ///< Grid cell size (m); overridden by octree resolution.
  // ── Boston Dynamics Spot physical dimensions ─────────────────────────────
  // Body: 1.1 m (L) × 0.5 m (W) × 0.6 m standing height.
  // Nominal ground-to-body clearance: ~0.17 m.
  // Circumscribed footprint radius: sqrt(0.55² + 0.25²) ≈ 0.60 m.
  // Max slope (BD spec): 30°; planner is tuned to handle up to 50°.
  double robot_height_{0.65};  ///< Vertical clearance needed: Spot body height (0.60 m) + 0.05 m margin.
  double robot_radius_{0.55};  ///< Conservative horizontal radius: slightly below Spot's circumscribed radius (0.60 m).
  double max_ray_drop_{3.0};   ///< Maximum downward ray length to find floor (m).
  /// Maximum terrain slope the robot can traverse (degrees).
  /// The clearance scan Z offset is derived automatically: resolution * tan(angle).
  /// At 30° and 0.1 m resolution → 0.058 m; at 50° → 0.119 m.
  double max_slope_deg_{30.0};
  // ── Anomaly / disturbance filter ─────────────────────────────────────────
  // A voxel is treated as an anomaly (noise, leg tip, transient object) if it
  // has fewer than anomaly_min_neighbors_ occupied face-neighbors (6-connected).
  // Anomaly voxels are ignored in both ground-finding and clearance checks.
  bool   anomaly_filter_enabled_{true};
  int    anomaly_min_neighbors_{2};  ///< Minimum occupied 6-neighbors to consider a voxel real.
  /// Cost multiplier applied to vertical (z) displacement between adjacent cells.
  /// Higher values make the planner prefer flatter routes.
  double z_penalty_factor_{10.0};
};

}  // namespace astar_octo_planner
