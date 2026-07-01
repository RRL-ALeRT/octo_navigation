cmake_minimum_required(VERSION 3.8)
project(graph_exploration)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  mbf_msgs
  alert_msgs
  octomap_msgs
  octomap_ros
)
foreach(dep IN LISTS dependencies)
  find_package(${dep} REQUIRED)
endforeach()

find_package(octomap REQUIRED)

include_directories(include ${OCTOMAP_INCLUDE_DIRS})

add_executable(graph_exploration_node
  src/graph_explorer.cpp
  src/graph_exploration_node.cpp
)

ament_target_dependencies(graph_exploration_node ${dependencies})
target_link_libraries(graph_exploration_node ${OCTOMAP_LIBRARIES})

install(TARGETS graph_exploration_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
