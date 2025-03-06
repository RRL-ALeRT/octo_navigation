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

## DSL
DSL is the required Library for for DSP ros2 path planning node.

Clone this into ~/Programs. Its not part of a ros2_ws: https://github.com/RRL-ALeRT/dsl/tree/dsp_ros2

    git clone -b "dsp_ros2" git@github.com:RRL-ALeRT/dsl.git
    cd dsl
    mkdir build
    cd build
    cmake ..
    sudo make install

Alternatively:

    git clone https://github.com/jhu-asco/dsl.git
    cd dsl
    git checkout 61cf588668309e87de209cd95f03a0f792a16c33
    mkdir build
    cd build
    cmake ..
    sudo make install

If you face the assert error change 0 to nullptr.

### Building
Clone octo_navigation with submodules:

    git clone --recurse-submodules git@github.com:RRL-ALeRT/octo_navigation.git

if already cloned:

    git submodule update --init --recursive

Clone mbf and `octomap_mapping` in ~/octo_nav_ws/src/ and colcon build.

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

D* of DSP is for UAV. Need support for UGV.

DSP Path Planning: https://github.com/LTU-RAI/Dsp/tree/ros2_msg

D*+: A Risk Aware Platform Agnostic Heterogeneous Path Planner: https://arxiv.org/pdf/2112.05563