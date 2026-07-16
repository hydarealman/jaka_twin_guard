cmake_minimum_required(VERSION 3.8)
project(jaka_single_arm)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package via setup.py
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/jaka_single_arm
)

# Install entry point scripts
install(
  PROGRAMS
    scripts/pick_place_runner
    scripts/fruit_detector_node
    scripts/fruit_target_node
    scripts/hand_eye_static_tf
    scripts/serial_trajectory_controller
    scripts/serial_fruit_target_bridge
    scripts/serial_board_emulator
    scripts/gazebo_scene_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Install config, launch, worlds, models to share
install(
  DIRECTORY config launch worlds models docs
  DESTINATION share/${PROJECT_NAME}
)

# Install BT tree XML to share
install(
  DIRECTORY jaka_single_arm/behavior/trees
  DESTINATION share/${PROJECT_NAME}/behavior
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_serial_protocol test/test_serial_protocol.py)
  ament_add_pytest_test(test_serial_integration test/test_serial_integration.py)
  ament_add_pytest_test(test_target_tracker test/test_target_tracker.py)
  ament_add_pytest_test(test_object_detector test/test_object_detector.py)
  ament_add_pytest_test(test_bt_reset test/test_bt_reset.py)
endif()

ament_package()
