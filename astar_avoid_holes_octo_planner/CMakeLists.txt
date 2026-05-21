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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)




add_Library(astar_avoid_holes_octo_planner SHARED src/astar_avoid_holes_octo_planner.cpp)
target_include_directories(astar_avoid_holes_octo_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(astar_avoid_holes_octo_planner PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
astar_avoid_holes_octo_planner  
rclcpp
mbf_octo_core
mbf_msgs
mbf_utility
pluginlib
octomap
octomap_msgs
visualization_msgs)
pluginlib_export_plugin_description_file(mbf_octo_core plugins.xml)
target_include_directories(astar_avoid_holes_octo_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rclcpp_INCLUDE_DIRS}
)

target_link_libraries(astar_avoid_holes_octo_planner
  ${octomap_LIBRARIES}
)

install(TARGETS astar_avoid_holes_octo_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
