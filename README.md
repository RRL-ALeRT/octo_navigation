# octo_navigation
Real-time 3D Navigation based on OctoMaps for unstructured environments.

![ALeRT 3D Nav](https://https://github.com/RRL-ALeRT/octo_navigation/blob/main/alert_3dnav.png)

## Installation

### Dependencies

move base flex:

Fork: https://github.com/METEORITENMAX/move_base_flex/tree/humble

Originally:
https://github.com/naturerobots/move_base_flex

octomapping:

https://github.com/RRL-ALeRT/octomap_mapping/tree/feature/global_and_local_mapping

  $ ros2 launch octomap_server octomap_webots_launch.py

### Building
Clone:
  octomap_mapping
  Move Base Flex
  octo_navigation

in a `ros2` workspace and `colcon build`


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


Add alert_rviz_plugin in Rviz2:

Click Panels -> Add new Panel -> AlertPanel


### Send a Goal
#### RViz2
Use ` 3DPoseEstimate` arrow to send a goal and click `Exec Path` button.

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
