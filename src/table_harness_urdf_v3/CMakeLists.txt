cmake_minimum_required(VERSION 3.8)

project(table_harness_urdf_v3)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
