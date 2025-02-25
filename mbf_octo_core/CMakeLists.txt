cmake_minimum_required(VERSION 3.8)
project(mbf_octo_core)

find_package(ament_cmake_ros REQUIRED)
set(dependencies
  mbf_abstract_core
)

foreach(dep IN LISTS dependencies)
  find_package(${dep} REQUIRED)
endforeach()

include_directories(
  include
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()