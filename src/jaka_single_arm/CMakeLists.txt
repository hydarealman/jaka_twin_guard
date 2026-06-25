cmake_minimum_required(VERSION 3.8)
project(jaka_single_arm)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package via setup.py
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/jaka_single_arm
)

# Install entry point script
install(
  PROGRAMS
    scripts/pick_place_runner
  DESTINATION lib/${PROJECT_NAME}
)

# Install config, launch, worlds to share
install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install BT tree XML to share
install(
  DIRECTORY jaka_single_arm/behavior/trees
  DESTINATION share/${PROJECT_NAME}/behavior
)

ament_package()
