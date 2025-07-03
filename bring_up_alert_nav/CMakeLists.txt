cmake_minimum_required(VERSION 3.8)
project(bring_up_alert_nav)

find_package(ament_cmake_ros REQUIRED)
# find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(mbf_octo_core REQUIRED)
find_package(mbf_msgs REQUIRED)
find_package(mbf_utility REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)  # ExePath goal.path
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED) # For StartNav.srv
find_package(webots_spot_msgs REQUIRED) # For posture

# Create Cpp executable
add_executable(exe_path_node src/exe_path_node.cpp)
ament_target_dependencies(exe_path_node
  rclcpp
  rclcpp_action
  std_srvs
  nav_msgs
  mbf_msgs
  mbf_octo_core
  mbf_utility
  geometry_msgs)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StartNav.srv"
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

install(
  TARGETS
    exe_path_node
    posture_manager
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python executables
install(
  DIRECTORY 
    launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Install params
install(
  DIRECTORY
    params
  DESTINATION share/${PROJECT_NAME}
)

# Export rosidl
ament_export_dependencies(
  rosidl_default_runtime
  mbf_octo_core 
  mbf_msgs 
  mbf_utility 
  rclcpp pluginlib 
  clcpp_action 
  geometry_msgs)

rosidl_get_typesupport_target(TYPESUPPORT_CPP_TARGET ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(exe_path_node ${TYPESUPPORT_CPP_TARGET})

ament_package()
