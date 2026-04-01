cmake_minimum_required(VERSION 3.8)
project(alert_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

find_package(ament_cmake_ros REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mbf_msgs REQUIRED)
find_package(bring_up_alert_nav REQUIRED)
find_package(std_srvs REQUIRED)

qt5_wrap_cpp(MOC_FILES
  include/alert_nav_plugins/alert_panel.hpp
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/alert_panel.cpp
  ${MOC_FILES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
)

ament_target_dependencies(${PROJECT_NAME}
  rviz_common
  rclcpp
  rclcpp_action
  pluginlib
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  mbf_msgs
  bring_up_alert_nav
  std_srvs
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugins_description.xml
  DESTINATION share/${PROJECT_NAME}
  RENAME plugins_description.xml
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_package()