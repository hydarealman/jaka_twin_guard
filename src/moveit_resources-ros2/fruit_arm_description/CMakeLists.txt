cmake_minimum_required(VERSION 3.8)
project(fruit_arm_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
