# octo_navigation
Real-time 3D Navigation based on OctoMaps for unstructured environments.

![ALeRT 3D Nav](https://github.com/RRL-ALeRT/octo_navigation/blob/main/alert_3dnav.png)

## Installation

### Dependencies

| Package | Repository | Branch |
|---|---|---|
| Move Base Flex | [METEORITENMAX/move_base_flex](https://github.com/METEORITENMAX/move_base_flex/tree/humble) (fork of [naturerobots/move_base_flex](https://github.com/naturerobots/move_base_flex)) | `humble` |
| OctoMap Mapping | [RRL-ALeRT/octomap_mapping](https://github.com/RRL-ALeRT/octomap_mapping/tree/feature/global_and_local_mapping) | `feature/global_and_local_mapping` |
| ALeRT Messages | [RRL-ALeRT/alert_msgs](https://github.com/RRL-ALeRT/alert_msgs) | `main` |

> **Note:** Move Base Flex must be on the `humble` branch of the fork.

All source dependencies are listed in [`alert_nav.repos`](alert_nav.repos) with their correct branches.

### Building

Install the tools (once):

```bash
sudo apt install python3-vcstool python3-rosdep
sudo rosdep init   # skip if already initialized
rosdep update
```

Clone this repository into the `src` folder of a ROS 2 workspace and import the dependencies with `vcs`:

```bash
mkdir -p ~/octo_nav_ws/src && cd ~/octo_nav_ws/src

git clone git@github.com:RRL-ALeRT/octo_navigation.git
vcs import < octo_navigation/alert_nav.repos
```

Install the system dependencies with `rosdep`:

```bash
cd ~/octo_nav_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

Build the workspace:

```bash
cd ~/octo_nav_ws
colcon build
source install/setup.bash
```

To update all dependency repositories later:

```bash
cd ~/octo_nav_ws/src
vcs import < octo_navigation/alert_nav.repos
vcs pull
```


## Start

Start in different terminals:

`ros2 launch webots_spot spot_launch.py`

`ros2 launch webots_spot octo_nav_launch.py`
or
`ros2 launch bring_up_alert_nav alert_nav_launch.py`

`ros2 launch octomap_server octomap_webots_launch.py`

`rviz2`

`ros2 run bring_up_alert_nav offset_tf_pub   --ros-args   --params-file /home/<username>/octo_nav_ws/src/octo_navigation/bring_up_alert_nav/params/offset_frames.yaml`


  Note: make sure the config `.yaml` suits your robot. By default `mbf_alert_nav.yaml` is loaded.


### RViz2

A preconfigured RViz2 config is provided in `alert_rviz_plugins`:

```bash
rviz2 -d ~/octo_nav_ws/src/octo_navigation/alert_rviz_plugins/rviz2/alert_nav.rviz
```

Add the `alert_rviz_plugin` panel in RViz2: click **Panels → Add New Panel → AlertPanel**.

#### Important topics

| Display type | Topic | Description |
|---|---|---|
| OccupancyGrid (`octomap_rviz_plugins`) | `/octomap_binary_local` | Local OctoMap |
| OccupancyGrid (`octomap_rviz_plugins`) | `/octomap_binary_full` | Full (global) OctoMap |
| MarkerArray | `/move_base_flex/graph_nodes` | Planning graph. Namespace `graph_nodes` shows the walkable nodes, `graph_penalty` shows the penalty nodes |
| Path | `/move_base_flex/body_height/path` | Planned path |

> **Note:** If the `octomap_rviz_plugins/OccupancyGrid` display type is not available in RViz2, install the plugin:
>
> ```bash
> sudo apt install ros-humble-octomap-rviz-plugins
> ```


### Send a Goal
#### RViz2
Use `2DPoseEstimate` arrow to send a goal and click `Exec Path` button.

#### Terminal
Type `ros2 action send_goal /move_base_flex/move_base mbf_msgs/action/MoveBase "t<tab>`

The message is autocompleted when typing " and the letter of the first arg of the message.

```
$ ros2 action send_goal /move_base_flex/move_base mbf_msgs/action/MoveBase "target_pose:
  header:
    stamp:
      sec: 0
      nanosec: 0
    frame_id: ''
  pose:
    position:
      x: 3.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
controller: ''
planner: ''
recovery_behaviors: []"
```

## Webots Rescue Arena

webots spot `rescue_arena` branch:
https://github.com/MASKOR/webots_ros2_spot/tree/rescue_arena

If you use our rescue arena in webots, use the launch file inside `webots_spot` to start this navigation with the correct `params`.

## Confige yaml
Example `.yaml` for our webots world:

    move_base_flex:
      ros__parameters:
        global_frame: 'map'
        robot_frame: 'base_footprint'
        odom_topic: '/Spot/odometry'

        use_sim_time: true
        force_stop_at_goal: true
        force_stop_on_cancel: true

        planners: ['octo_planner']
        octo_planner:
          type: 'astar_octo_planner/AstarOctoPlanner'
          cost_limit: 0.8
          publish_vector_field: true
          octomap_topic: '/octomap_binary_local'
        planner_patience: 10.0
        planner_max_retries: 2

        controllers: ['octo_controller']
        octo_controller:
          type: 'octo_controller/OctoController'
          ang_vel_factor: 0.25
          lin_vel_factor: 0.35

        controller_patience: 2.0
        controller_max_retries: 4
        dist_tolerance: 0.2
        angle_tolerance: 0.8
        cmd_vel_ignored_tolerance: 10.0

        controller_frequency: 10.0
