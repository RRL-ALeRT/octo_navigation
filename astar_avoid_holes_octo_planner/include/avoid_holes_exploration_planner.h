#ifndef OCTO_NAVIGATION__AVOID_HOLES_EXPLORATION_PLANNER_H
#define OCTO_NAVIGATION__AVOID_HOLES_EXPLORATION_PLANNER_H

#include "astar_avoid_holes_octo_planner.h"

namespace astar_planner {

// An MBF planner plugin that explores instead of going to a fixed goal.
//
// It reuses the avoid-holes planner's graph reformation (prepareGraph: prune + fill holes
// + inflate) and A* (runAstar). The incoming goal is ignored; the plugin:
//   1. reforms the graph exactly like a normal plan (so penalties/holes are correct),
//   2. picks the walkable node with the most UNKNOWN cells around it (the frontier), across
//      all z-levels present in the graph,
//   3. BFS-moves that frontier to the nearest low-penalty ("safe") node,
//   4. runs A* from the robot to that safe node and returns the path.
class AvoidHolesExplorationPlanner : public AstarAvoidHolesOctoPlanner {
public:
    typedef std::shared_ptr<AvoidHolesExplorationPlanner> Ptr;

    // Sets up the base publishers (graph_octomap / penalty_octomap / path) plus this
    // plugin's own frontier_markers publisher.
    bool initialize(const std::string & name,
                    const rclcpp::Node::SharedPtr & node,
                    const mbf_octo_core::OctoMapper::Ptr & mapper) override;

    uint32_t makePlan(const geometry_msgs::msg::PoseStamped & start,
                      const geometry_msgs::msg::PoseStamped & goal,
                      double tolerance,
                      std::vector<geometry_msgs::msg::PoseStamped> & plan,
                      double & cost,
                      std::string & message) override;

private:
    // Walkable node with the most unknown (octree == nullptr) cells around it, or "" if
    // none reaches min_unknown. Works across every z-level in the graph. As a side effect
    // records every candidate (unknown >= min_unknown_) in last_frontier_candidates_ so
    // publishFrontierMarkers can draw the whole frontier field.
    std::string selectFrontierNode(const std::shared_ptr<mbf_octo_core::GraphData>& graph,
                                   const octomap::OcTree* octree);

    // Publish a MarkerArray on frontier_markers: every candidate frontier (small spheres,
    // brighter = more unknown), the SELECTED frontier (big magenta), the chosen goal
    // (green), and an arrow from frontier to goal.
    void publishFrontierMarkers(const std::shared_ptr<mbf_octo_core::GraphData>& graph,
                                const std::string& frontier_node,
                                const std::string& goal_node);

    // BFS out from the frontier over walkable nodes and return the CLOSEST node whose
    // penalty is at/below acceptable_penalty_ - i.e. pull back only as far as needed to
    // reach a safe (low-penalty) node. If no node under the cap is reachable, fall back
    // to the lowest-penalty node seen. Always returns a node (never "").
    std::string nearestLowPenaltyNode(const std::shared_ptr<mbf_octo_core::GraphData>& graph,
                                      const std::string& from);

    // Candidate frontiers from the most recent selectFrontierNode call: (node id, unknown
    // count). Used purely for visualization.
    std::vector<std::pair<std::string, int>> last_frontier_candidates_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontier_marker_pub_;

    // Exploration floor: height latched on the FIRST request and kept as the lowest ever
    // seen. Graph pruning uses this instead of the robot's current height, so climbing a
    // platform doesn't prune the lower map away.
    bool   has_floor_z_        = false;
    double exploration_floor_z_ = 0.0;

    // --- tunables ---
    int    unknown_radius_        = 2;      // cells to scan around a node for unknown space
    int    min_unknown_           = 1;      // min unknown cells for a node to be a frontier
    double acceptable_penalty_    = 500.0;  // goal pulls back until penalty <= this ("safe").
                                            // Scale matches inflateLevel's 30000/2^step, so
                                            // ~step 6 off the edge. Lower = pull back further.
    double heading_weight_        = 0.6;    // 0..1 blend: 0 = pure most-unknown, 1 = pure
                                            // "expand where Spot is looking" (forward bias)
    double max_frontier_dist_     = 3.0;    // m; frontiers beyond this are last-resort only
    double front_cone_half_deg_   = 60.0;   // deg; half-angle of the "ahead or slightly to
                                            // the side" cone. Near+in-cone frontiers are
                                            // ALWAYS preferred over side/behind/far ones.
    double frontier_ring_step_    = 0.5;    // m; radius-expansion ring width. The nearest
                                            // non-empty ring wins, expanding outward up to
                                            // max_frontier_dist_.
    int    max_frontier_attempts_ = 30;     // max distinct frontiers to A*-test per plan
    double frontier_cluster_dist_ = 0.5;    // m; frontiers closer than this are one region
    double min_travel_dist_       = 0.6;    // m; skip a frontier whose goal is nearer than
                                            // this to the robot (no real progress -> would loop)
};

} // namespace astar_planner

#endif
