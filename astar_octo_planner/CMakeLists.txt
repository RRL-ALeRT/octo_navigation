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
