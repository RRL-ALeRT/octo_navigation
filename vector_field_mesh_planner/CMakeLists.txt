cmake_minimum_required(VERSION 3.8)
project(vector_field_mesh_planner)

find_package(ament_cmake_ros REQUIRED)
find_package(mbf_octo_core REQUIRED)
find_package(mbf_msgs REQUIRED)
find_package(mbf_utility REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(mesh_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

pluginlib_export_plugin_description_file(mbf_octo_core vector_field_mesh_planner.xml)

add_library(${PROJECT_NAME}
  src/vector_field_mesh_planner.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${pluginlib_INCLUDE_DIRS}
)
target_compile_definitions(${PROJECT_NAME} PRIVATE "VECTOR_FIELD_MESH_PLANNER_BUILDING_LIBRARY")
ament_target_dependencies(${PROJECT_NAME}
  mbf_octo_core mbf_msgs mbf_utility rclcpp pluginlib
  mesh_msgs nav_msgs geometry_msgs visualization_msgs
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

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(mbf_octo_core mbf_msgs mbf_utility rclcpp pluginlib mesh_msgs nav_msgs geometry_msgs visualization_msgs)
ament_package()
