cmake_minimum_required(VERSION 3.8)
project(jaka_dual_arm)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package via setup.py
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/jaka_dual_arm
)

# Install entry point scripts
install(
  PROGRAMS
    scripts/carry_task_runner
    scripts/massage_runner
  DESTINATION lib/${PROJECT_NAME}
)

# Install config, launch, worlds to share
install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
