cmake_minimum_required(VERSION 3.8)
project(octo_controller)


find_package(ament_cmake_ros REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(mbf_octo_core REQUIRED)
find_package(mbf_msgs REQUIRED)
find_package(mbf_utility REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(MPI)


pluginlib_export_plugin_description_file(mbf_octo_core octo_controller.xml)

add_library(${PROJECT_NAME} src/octo_controller.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${pluginlib_INCLUDE_DIRS}
)
target_compile_definitions(${PROJECT_NAME} PRIVATE "OCTO_CONTROLLER_BUILDING_LIBRARY")
ament_target_dependencies(${PROJECT_NAME} example_interfaces mbf_octo_core mbf_msgs mbf_utility rclcpp tf2_geometry_msgs pluginlib visualization_msgs)
target_link_libraries(${PROJECT_NAME}
    ${MPI_CXX_LIBRARIES}
)

install(DIRECTORY include DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(example_interfaces mbf_octo_core mbf_msgs mbf_utility rclcpp tf2_geometry_msgs pluginlib visualization_msgs)
ament_package()
