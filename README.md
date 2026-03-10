# octo_navigation

## Installation

### Dependencies

move base flex:
https://github.com/METEORITENMAX/move_base_flex
or
https://github.com/naturerobots/move_base_flex

octomapping:

https://github.com/RRL-ALeRT/octomap_mapping/feature/global_and_local_mapping

webots spot mbf octo branch:
https://github.com/MASKOR/webots_ros2_spot/tree/mbf_octo_nav


### Building
Clone octo_navigation: https://github.com/RRL-ALeRT/octo_navigation
and
mbf and in one workspace and colcon build.

## Start

Start in different terminals:

`ros2 launch webots_spot spot_launch.py`

`ros2 launch webots_spot octo_nav_launch.py`

`ros2 launch octomap_server octomap_webots_launch.py`

`rviz2`

`ros2 run bring_up_alert_nav offset_tf_pub   --ros-args   --params-file /home/<username>/octo_nav_ws/src/octo_navigation/bring_up_alert_nav/params/offset_frames.yaml`

Add alert_rviz_plugin in Rviz2:

Click Panels -> Add new Panel -> AlertPanel

Use Octomap updates Button to turn on/off octomap subscription.

### Send a Goal
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
