cmake_minimum_required(VERSION 3.8)
project(mbf_octo_nav)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake_ros REQUIRED)
# ROS deps
set(dependencies
  geometry_msgs
  mbf_abstract_nav
  mbf_simple_core
  mbf_octo_core
  mbf_msgs
  nav_msgs
  pluginlib
  rclcpp
  std_srvs
)
foreach(dep IN LISTS dependencies)
  find_package(${dep} REQUIRED)
endforeach()

include_directories(
  include
)

find_package(MPI)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

include_directories(
  include
)

add_library(mbf_octo_server
  src/octo_controller_execution.cpp
  src/octo_navigation_server.cpp
  src/octo_planner_execution.cpp
  src/octo_recovery_execution.cpp
)
target_include_directories(mbf_octo_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mbf_octo_server ${dependencies})
target_link_libraries(mbf_octo_server
  ${JSONCPP_LIBRARIES}
  ${MPI_CXX_LIBRARIES}
)

add_executable(${PROJECT_NAME} src/mbf_octo_nav.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}
  ${MPI_CXX_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  mbf_octo_server
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS mbf_octo_server
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()