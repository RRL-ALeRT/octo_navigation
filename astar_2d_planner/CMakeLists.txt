cmake_minimum_required(VERSION 3.8)
project(astar_2d_planner)

find_package(ament_cmake_ros REQUIRED)
find_package(mbf_octo_core REQUIRED)
find_package(mbf_msgs REQUIRED)
find_package(mbf_utility REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)  # ExePath goal.path
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED) # For StartNav.srv

pluginlib_export_plugin_description_file(mbf_octo_core astar_2d_planner.xml)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StartNav.srv"
)

# Export rosidl
ament_export_dependencies(rosidl_default_runtime)

# Define using core library as <project>_core
add_library(${PROJECT_NAME}_core SHARED
  src/astar_2d_planner.cpp
)
ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp 
  mbf_octo_core 
  mbf_msgs 
  mbf_utility 
  nav_msgs 
  geometry_msgs
  pluginlib
  sensor_msgs
  pcl_conversions
  tf2_geometry_msgs
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${pluginlib_INCLUDE_DIRS}
)
target_compile_definitions(${PROJECT_NAME}_core PRIVATE 
  "astar_2d_planner_BUILDING_LIBRARY"
)

target_link_libraries(${PROJECT_NAME}_core
  ${MPI_CXX_LIBRARIES}
)

# Define for exe_path_after_node
add_executable(exe_path_after_node 
  src/exe_path_node.cpp
)

ament_target_dependencies(exe_path_after_node
  rclcpp 
  rclcpp_action
  std_srvs 
  nav_msgs 
  mbf_msgs 
  mbf_octo_core
  mbf_utility
  geometry_msgs
)

# Two path
# ROSIDL typesupport to exe
# rosidl_target_interfaces(exe_path_after_node "rosidl_typesupport_cpp")
rosidl_get_typesupport_target(TYPESUPPORT_CPP_TARGET ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(exe_path_after_node ${TYPESUPPORT_CPP_TARGET})

# install rule
install(
  TARGETS
    ${PROJECT_NAME}_core
    exe_path_after_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  EXPORT export_${PROJECT_NAME}
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

# install plugin XML
install(
  FILES astar_2d_planner.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# export for ament
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  mbf_octo_core 
  mbf_msgs 
  mbf_utility 
  rclcpp 
  pluginlib
  std_srvs
  nav_msgs
  geometry_msgs
  )
ament_package()
