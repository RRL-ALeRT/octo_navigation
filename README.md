# octo_navigation

## Installation

### Dependencies

move base flex:
https://github.com/METEORITENMAX/move_base_flex
or
https://github.com/naturerobots/move_base_flex

octomapping:

https://github.com/RRL-ALeRT/octomap_mapping

webots spot mbf octo branch:
https://github.com/MASKOR/webots_ros2_spot/tree/mbf_octo_nav

## DSP

https://github.com/LTU-RAI/Dsp/tree/ros2_msg

    cd
    git clone https://github.com/jhu-asco/dsl.git
    cd dsl
    git checkout 61cf588668309e87de209cd95f03a0f792a16c33
    mkdir build
    cd build
    cmake ..
    sudo make install

If you face the assert error change 0 to nullptr.

### Building
Clone octo_navigation: https://github.com/RRL-ALeRT/octo_navigation
and
mbf and in one workspace and colcon build.

## Start

Start in different terminals:

`ros2 launch webots_spot spot_launch.py`

`ros2 launch webots_spot octo_nav_launch.py`

`ros2 launch octomap_server octomap_webots_launch.py`

`ros2 run dsp dsp`

`ros2 run pure_pursuit_controller path_to_vel`

`rviz`

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

## RVIZ

Add tfs, RobotModel .etc and:
PointCloud2: `/octomap_point_cloud_centers`
Path: `/move_base_flex/path`
If you want to see the octomap:
OccupancyGrid: `/octomap_binary`
## Issues

Astar not finding correct path, fix: `make_plan_service/path_planner.py`

Octo Controller does not stop when reaching goal, fix: `pure_pursuit_controller/path_to_vel.py` and octo controller to inform each other if target reached

DSP Path Planning: https://github.com/LTU-RAI/Dsp/tree/ros2_msg