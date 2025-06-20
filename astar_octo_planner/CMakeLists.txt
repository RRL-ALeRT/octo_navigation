cmake_minimum_required(VERSION 3.8)
project(astar_octo_planner)

find_package(ament_cmake_ros REQUIRED)
find_package(mbf_octo_core REQUIRED)
find_package(mbf_msgs REQUIRED)
find_package(mbf_utility REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(webots_spot_msgs REQUIRED)

pluginlib_export_plugin_description_file(mbf_octo_core astar_octo_planner.xml)


add_library(${PROJECT_NAME}
  src/astar_octo_planner.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${pluginlib_INCLUDE_DIRS}
)
target_compile_definitions(${PROJECT_NAME} PRIVATE "ASTAR_OCTO_PLANNER_BUILDING_LIBRARY")
ament_target_dependencies(${PROJECT_NAME} 
  mbf_octo_core 
  mbf_msgs 
  mbf_utility 
  rclcpp 
  pluginlib 
  sensor_msgs 
  pcl_conversions)
target_link_libraries(${PROJECT_NAME}
    ${MPI_CXX_LIBRARIES}
)

install(DIRECTORY include/
 DESTINATION include
)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# new node
add_executable(rviz_goal_sender_node src/rviz_goal_sender_node.cpp) 
ament_target_dependencies(rviz_goal_sender_node
  rclcpp
  rclcpp_action
  geometry_msgs
  mbf_msgs
)
install(TARGETS rviz_goal_sender_node
 DESTINATION lib/${PROJECT_NAME}
)

# new service
add_executable(posture_manager src/posture_manager.cpp)
ament_target_dependencies(posture_manager
  rclcpp 
  nav_msgs 
  geometry_msgs 
  mbf_octo_core 
  mbf_msgs 
  mbf_utility 
  webots_spot_msgs)
install(TARGETS posture_manager
 DESTINATION lib/${PROJECT_NAME}
)



find_package(ament_cmake_auto REQUIRED)


ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  mbf_octo_core 
  mbf_msgs 
  mbf_utility 
  rclcpp pluginlib 
  clcpp_action 
  geometry_msgs)
ament_package()
