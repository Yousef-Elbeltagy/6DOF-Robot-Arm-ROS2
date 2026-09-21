cmake_minimum_required(VERSION 3.8)

project(robot_arm_with_gripper_urdf)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
