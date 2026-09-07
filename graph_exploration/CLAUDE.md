# graph_exploration — CLAUDE.md

## Role

ROS 2 node (`graph_explorer`) that performs goal-directed frontier exploration using
Move Base Flex (MBF) as the underlying navigation stack. Given a goal pose on
`/alert_exploration_pose`, it autonomously picks intermediate waypoints until the goal
is reached. The node is completely external to MBF — it only uses the `move_base`
action and reads two auxiliary topics published by `OctoMappingServer`.

---

## Key files

- `include/graph_exploration/graph_explorer.hpp` — class definition
- `src/graph_explorer.cpp` — all logic
- `src/graph_exploration_node.cpp` — `main()`, spins with `MultiThreadedExecutor`

**MultiThreadedExecutor is required** so subscription callbacks (octomap, graph cloud,
goal pose, action results) fire while the exploration thread blocks waiting for MBF.

---

## Topics

| Direction | Topic | Type | Notes |
|---|---|---|---|
| Sub | `/alert_exploration_pose` | `geometry_msgs/PoseStamped` | New goal, clears visited list |
| Sub | `/move_base_flex/graph_cloud` | `sensor_msgs/PointCloud2` | Walkable graph nodes from `OctoMappingServer`; transient-local so latest cloud arrives immediately on startup |
| Sub | `octomap_topic` param (default `/octomap_binary`) | `octomap_msgs/Octomap` | Full accumulated octomap used for frontier detection and raycasting |
| Action client | `move_base_flex/move_base` | `mbf_msgs/MoveBase` | Each waypoint sent with `planner="octo_planner"`, `controller="octo_controller"` |

---

## Graph cloud format

Published by `OctoMappingServer` on `~/graph_cloud` (transient-local).  
`point_step = 20`, fields per point (all `float32`):

| Offset | Field |
|---|---|
| 0 | x |
| 4 | y |
| 8 | z |
| 12 | penalty (wall-proximity cost, 0 = green, high = red) |
| 16 | neighbor_count (walkable adjacency count; low = corner / edge of graph) |

---

## Exploration algorithm

### High-level loop (`explore()`)

Each iteration:
1. Get robot pose via TF (`map_frame_` → `robot_frame_`).
2. **Goal reached** check: if XY distance to goal < `goal_tolerance_` (0.3 m) → done.
3. **Direct send** check: if XY distance to goal < 0.5 m → send goal directly to MBF → done.
4. **Goal in graph** check: if any walkable graph node is within 0.5 m XY of the goal → send goal directly to MBF → done. This fires once the robot has explored close enough for the planner to see the goal.
5. Call `findCandidates()` → get frontier-aligned graph nodes + clearance scores.
6. Call `selectBest()` → pick highest-scoring candidate.
7. Send MBF goal to waypoint; blacklist waypoint regardless of success/failure.
8. After 3 consecutive empty candidate cycles → send final goal as last resort.

### Frontier detection (`findCandidates()`)

Uses the **full octomap** (not just local radius) for true frontier detection:

**Step 1 — Global frontier voxels**
Iterates all free leaf voxels in the full octomap bounding box (Z: robot ± 1.5 m).
A voxel is a frontier if at least one of its 6 face-neighbours is unknown
(`tree.search()` returns `nullptr`).

Score per frontier voxel:
```
alignment  = dot(robot→frontier, robot→goal)   ∈ [−1, 1]
open_score = 1 − occupied_face_neighbours / 6  ∈ [0, 1]
score      = alignment × open_score
```
**All frontier directions are considered** — including behind the robot — so
the robot can backtrack to unexplored pockets rather than only heading forward.
Arena outer walls (many occupied neighbours) still score low; open passages score high.

**Step 2 — Exploration bearing**
Top-50 frontier voxels (by score) are kept. Their unit directions from the robot
are averaged → one "exploration bearing" vector. Falls back to the straight
robot→goal direction if no frontiers are found (fully explored environment).

**Step 3 — Filter local graph nodes**
Graph nodes within `frontier_radius_` (2.0 m) of the robot that are broadly aligned
with the exploration bearing (dot > 0.05) become candidates.
Fallback: if the bearing filter empties the list, all local nodes in radius are used.

**Step 4 — Clearance raycasts**
For every candidate, `computeClearance()` casts 12 rays:
- 4 horizontal directions (±x, ±y in map frame)
- 3 heights above the node: z + 0.5 m, z + 0.7 m, z + 0.9 m (covering robot body)
- Each ray up to `kRayMaxDist` = 2.0 m; ignores unknown voxels
- Returns the **minimum** per-direction clearance (avg of 3 heights per direction)

All candidates receive raycasts — no pre-filter. The minimum directional value
means a single nearby wall tanks the score; only truly open nodes win.

### Candidate scoring (`selectBest()`)

```
score = 0.65 × norm_clearance        // primary: open space in all directions
      + 0.15 × alignment_to_goal     // secondary: directional progress
      + 0.10 × dist_bonus            // prefer outer edge of search radius
      − penalty_weight × norm_pen    // penalise wall-adjacent graph nodes
```

Candidates closer than `min_waypoint_dist_` (0.7 m) to the robot are skipped.
Candidates within `visited_radius_` (0.7 m) of any previously visited waypoint
are skipped. If all candidates are filtered, the visited list is cleared and the
front of the list is returned as emergency fallback.

---

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `frontier_radius` | 2.0 m | Search radius for local graph node candidates |
| `min_waypoint_dist` | 0.7 m | Ignore candidates closer than this to the robot |
| `goal_tolerance` | 0.3 m | XY distance at which the goal is considered reached |
| `penalty_weight` | 0.4 | Weight of graph node penalty in scoring |
| `visited_radius` | 0.7 m | Blacklist radius around visited waypoints |
| `robot_frame` | `base_link` | TF frame for robot pose |
| `map_frame` | `map` | TF frame for the map |
| `octomap_topic` | `/octomap_binary` | Full accumulated octomap for frontier detection and raycasting |

---

## Threading model

- **Main thread** (executor): spins callbacks — goal pose, graph cloud, octomap, MBF action results
- **Exploration thread**: blocks on MBF action `future.wait_for(100 ms)` in a poll loop; checks `shutdown_` and `goal_changed_` each tick; cancels active MBF goal with a 3 s drain timeout on interruption

Shared state protected by separate mutexes:
- `graph_mutex_` — `graph_nodes_`
- `octree_mutex_` — `octree_`
- `goal_mutex_` + `goal_cv_` — `final_goal_`, `has_goal_`
- `goal_handle_mutex_` — `active_goal_handle_`
- `visited_mutex_` — `visited_waypoints_`

`goal_changed_` and `shutdown_` are `std::atomic_bool` (lock-free).

---

## Design decisions

- **Full octomap vs. local**: Uses `/octomap_binary` (full map) for frontier detection so the exploration bearing reflects the entire arena, not just the 3 m local scan. The local 3 m graph cloud is used for actual navigation targets (walkability guaranteed).
- **Minimum clearance**: `computeClearance` returns the min of 4 directional averages, not the overall average. One nearby wall in any direction → low score, forcing selection of truly open nodes.
- **No path_penalty**: Earlier versions computed corridor penalties from graph nodes; replaced by actual octomap raycasting which is more accurate.
- **Visited blacklist cleared on new goal**: When a new `/alert_exploration_pose` arrives, the visited list is cleared so all waypoints in the new arena region are eligible.
- **Direct goal at 0.5 m / goal-in-graph**: Two independent early-exit conditions prevent the robot from spending time picking frontiers when it can already see the goal.
