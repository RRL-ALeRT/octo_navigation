# octo_controller — CLAUDE.md

## Role

MBF controller plugin (`mbf_octo_core::OctoController`) that converts a path into `cmd_vel` commands using pure pursuit.

## Key files

- `include/octo_controller/octo_controller.h` — class definition, `config_` struct
- `src/octo_controller.cpp` — all logic

---

## Algorithm overview

### computeVelocityCommands

Two modes depending on distance to goal:

| Condition | Mode |
|---|---|
| `goal_dist >= arrival_fading` | Pure pursuit via `purePursuit()` |
| `goal_dist < arrival_fading` | **Parking mode**: `v=0`, P-controller on heading error |

Parking mode uses bang-bang at `max_ang_velocity` until within the fine-alignment zone (`max_ang_velocity / ang_vel_factor`), then switches to `ang_vel_factor * angle_err`. A `parking_rotation_sign_` lock prevents sign flipping at the ±π boundary.

Orientation alignment is only enforced when the goal quaternion is non-identity (A* planner emits identity quaternions).

### purePursuit

Scans path from `pursuit_index_` forward to find the first point farther than `lookahead_distance` (`max_search_distance`). Determines forward/backward direction via dot product of lookahead vector with robot heading.

**When `backward_walking_enable = true`** (default):
- Lookahead behind robot → `v = -speed`, aim front opposite to target

**When `backward_walking_enable = false`**:
- Lookahead behind robot → `v = 0`, rotate in place to face target first

Angular velocity is clamped to `±M_PI/4`; linear velocity is zeroed when the steering angle exceeds `M_PI/6`.

---

## Parameters

All parameters are under the plugin name namespace (e.g. `octo_controller.<param>`).

| Parameter | Type | Default | Description |
|---|---|---|---|
| `max_lin_velocity` | double | 1.0 | Maximum linear speed (m/s) |
| `max_ang_velocity` | double | 1.0 | Maximum angular speed (rad/s) |
| `arrival_fading` | double | 0.5 | Distance to goal (m) that triggers parking mode |
| `ang_vel_factor` | double | 6.0 | P-gain for heading in parking mode; fine-zone = max_ang/factor |
| `lin_vel_factor` | double | 1.0 | Linear velocity scaling factor |
| `max_angle` | double | 20.0 | Maximum angle for linear velocity function (deg) |
| `max_search_radius` | double | 0.4 | Maximum radius for consecutive neighbour search |
| `max_search_distance` | double | 0.4 | Lookahead distance for pure pursuit |
| `backward_walking_enable` | bool | true | Allow driving in reverse. If false, robot rotates in place to face target before driving forward. |

All parameters support dynamic reconfiguration at runtime.

---

## Goal tolerance

`isGoalReached()` uses hardcoded values (`dist_tolerance = 0.56 m`, `angle_tolerance = 0.06 rad`), overriding whatever MBF passes in. Angle check is skipped for identity-quaternion goals.

---

## Design notes

- `parking_rotation_sign_` is reset to `0.0` on each `setPlan()` call and when the P-controller phase begins.
- `cancel_requested_` is an `std::atomic_bool`; safe to set from any thread.
- The `angle_pub_` topic (`~/current_angle`) is published for debug purposes.
