# octo_navigation — CLAUDE.md

## Architecture Overview

3D octomap-based navigation using Move Base Flex (MBF) on ROS 2 (Humble).
Map is a local 3 m-radius `octomap::OcTree`; planning is graph-based A*.

### Package layout

| Package | Role |
|---------|------|
| `mbf_octo_core` | Shared interfaces and data types (`OctoMapper`, `OctoPlanner`, `GraphData`, `GraphNode`) |
| `mbf_octo_nav` | Navigation server (`OctoNavigationServer`) + **mapping server** (`OctoMappingServer`) |
| `astar_octo_planner` | A* planner plugin — path search only, no map ownership |

### Dependency direction

```
mbf_abstract_core
      ↑
mbf_octo_core          (GraphData, OctoMapper interface, OctoPlanner interface)
   ↑          ↑
mbf_octo_nav      astar_octo_planner
(OctoMappingServer implements OctoMapper)
(OctoNavigationServer creates OctoMappingServer and injects it into every planner plugin)
```

No circular deps: `mbf_octo_nav` depends on `mbf_octo_core`; `astar_octo_planner` also depends only on `mbf_octo_core`.

---

## Responsibility split

### OctoMappingServer (`mbf_octo_nav`)
- Subscribes to octomap topic, owns `octomap::OcTree`
- Builds walkability graph (floor/wall/stair classification, multi-threaded)
- Incremental graph updates as new scans arrive (double-buffered)
- Computes penalty costmap (centroid-shift + graph-border + Dijkstra spread)
- Exposes graph + octree to planners via `OctoMapper` interface
- `getReadyGraph()`: revalidates walkability, computes pending penalties, returns a thread-safe snapshot

### AstarOctoPlanner (`astar_octo_planner`)
- Receives graph snapshot from `mapper_->getReadyGraph()`
- Receives octree from `mapper_->getOctree()` for live collision checks
- Runs A* on the graph using pre-computed `node_penalty` as traversal cost
- Live collision checks: `hasVerticalClearance`, `hasRadialClearanceAbove`, `isEdgeCollisionFree`
- Publishes debug markers via `mapper_->publishAdditionalMarkers()`

---

## Key files

### mbf_octo_core (shared interfaces)
- `include/mbf_octo_core/octo_graph_data.h` — `GraphNode` + `GraphData` structs
- `include/mbf_octo_core/octo_mapper.h` — `OctoMapper` abstract interface
- `include/mbf_octo_core/octo_planner.h` — `OctoPlanner` abstract interface
  - `initialize(name, node, mapper)` — mapper injected here

### mbf_octo_nav (server + mapping)
- `include/mbf_octo_nav/octo_mapping_server.h`
- `src/octo_mapping_server.cpp`
- `include/mbf_octo_nav/octo_navigation_server.h` — owns `OctoMappingServer::Ptr mapping_server_`
- `src/octo_navigation_server.cpp` — creates + initialises mapping server in constructor; injects into planner via `initialize(name, node_, mapping_server_)`

### astar_octo_planner (path planner plugin)
- `include/astar_octo_planner/astar_octo_planner.h`
- `src/astar_octo_planner.cpp`

---

## Graph data model

```cpp
// mbf_octo_core::GraphNode
struct GraphNode {
  octomap::OcTreeKey key;
  unsigned int depth;
  octomap::point3d center;
  double size;
  bool is_walkable;
  bool is_stair_step;
  std::string id() const;   // "k0_k1_k2_depth"
};

// mbf_octo_core::GraphData
struct GraphData {
  std::unordered_map<std::string, GraphNode>              nodes;
  std::unordered_map<std::string, std::vector<std::string>> adj;
  std::unordered_map<std::string, double>                 node_penalty;
  // ... penalty cache, incremental tracking sets
};
```

---

## Walkability classification (inside OctoMappingServer)

1. **Phase 1** — collect occupied leaf voxels from octree (single-threaded)
2. **Phase 2** — multi-threaded walkability checks:
   - `hasVerticalClearance(center, size, check_height)` — no occupancy above up to `check_height`
   - `hasFloorSupport(x, y, z, node_size)` — ring of neighbors at similar Z (rejects wall tops)
3. **Phase 3** — build adjacency (multi-threaded, direct-distance + `max_step_height_` Z tolerance)
4. **detectAndAugmentStairs()** — BFS for stair chains; promotes treads to `is_stair_step=true`

Revalidation + penalty computation happen in `getReadyGraph()` before the graph is returned.

---

## Parameters

### OctoMappingServer (`octo_mapping_server.*`)
Graph build: `voxel_size`, `z_threshold`, `max_surface_distance`, `max_step_height`,
`min_floor_support_ratio`, `floor_support_num_dirs`, `wall_proximity_height/radius/num_dirs/num_z`,
stair detection params (`stair_min/max_rise`, `stair_min_chain_length`, etc.)

Penalty: `wall_penalty_weight`, `corner_penalty_weight`, `corner_radius`,
`centroid_k/lambda/penalty_weight`, `penalty_spread_radius/factor`, `sector_bins/radius/peak_thresh`,
`worker_thread_limit`

Subscription: `octomap_topic`, `enable_octomap_updates`, `incremental_graph_build`
Sensor model: `octomap_prob_hit/miss/thres/clamp_min/max`

### AstarOctoPlanner (`<planner_name>.*`)
Robot collision: `robot_radius`, `robot_width`, `robot_length`, `robot_height`,
`footprint_margin`, `footprint_samples_x/y`, `min/max_vertical_clearance`

A* checks: `radial_clearance_num_dirs/z`, `enable_radial_clearance`, `enable_edge_collision_check`

Snapping: `max_z_above_query`

---

## Important design decisions

- **No circular deps**: `OctoMapper` and `GraphData` live in `mbf_octo_core` (shared), not `mbf_octo_nav`.
- **`max_step_height_` shared**: owned by mapper (drives graph build), exposed via `getMaxStepHeight()` so planner's live collision checks use the same value.
- **`graph_marker_pub_` owned by mapper**: planner sends plan-specific debug markers (dead ends, start/goal) via `mapper_->publishAdditionalMarkers()`.
- **Penalty computation in mapper**: `getReadyGraph()` handles revalidation + incremental penalty computation before returning the snapshot. Planner just reads `node_penalty`.
- **`clearOccupiedCylinderAround`**: uses `const_cast` on mapper's octree. This is intentional (temporary in-memory clear for start-pose leg artifacts); it does not persist.
