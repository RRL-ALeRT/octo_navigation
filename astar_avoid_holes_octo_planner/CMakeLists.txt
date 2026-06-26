cmake_minimum_required(VERSION 3.8)
project(astar_avoid_holes_octo_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mbf_octo_core REQUIRED)
find_package(mbf_msgs REQUIRED)
find_package(mbf_utility REQUIRED)
find_package(pluginlib REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FindFrontiers.srv"
  DEPENDENCIES geometry_msgs
)

add_Library(avoid_holes_planner_plugin SHARED src/astar_avoid_holes_octo_planner.cpp)
target_include_directories(avoid_holes_planner_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(avoid_holes_planner_plugin PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
avoid_holes_planner_plugin  
rclcpp
mbf_octo_core
mbf_msgs
mbf_utility
pluginlib
octomap
octomap_msgs
visualization_msgs
geometry_msgs
tf2_ros
tf2_geometry_msgs)
pluginlib_export_plugin_description_file(mbf_octo_core plugins.xml)
target_include_directories(avoid_holes_planner_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rclcpp_INCLUDE_DIRS}
)

target_link_libraries(avoid_holes_planner_plugin
  ${octomap_LIBRARIES}
)

install(TARGETS avoid_holes_planner_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

  add_executable(frontier_server src/frontier_server.cpp)

  target_include_directories(frontier_server PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OCTOMAP_INCLUDE_DIRS}
  )

  ament_target_dependencies(frontier_server
    rclcpp
    octomap_msgs
    geometry_msgs
    visualization_msgs
    tf2_ros
    tf2_geometry_msgs
  )

  target_link_libraries(frontier_server
    ${OCTOMAP_LIBRARIES}
  )

  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(frontier_server "${cpp_typesupport_target}")

  install(TARGETS frontier_server
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )

  add_executable(exploration_node src/exploration_node.cpp)

  target_include_directories(exploration_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(exploration_node
    rclcpp
    rclcpp_action
    mbf_msgs
    nav_msgs
    geometry_msgs
  )

  rosidl_get_typesupport_target(cpp_typesupport_target_exp ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(exploration_node "${cpp_typesupport_target_exp}")

  install(TARGETS exploration_node
    DESTINATION lib/${PROJECT_NAME}
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
