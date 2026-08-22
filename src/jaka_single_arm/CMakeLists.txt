cmake_minimum_required(VERSION 3.8)
project(jaka_single_arm)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(control_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS aruco calib3d core imgproc)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(eye_to_hand_solver
  src/eye_to_hand_solver.cpp
)
target_compile_features(eye_to_hand_solver PUBLIC cxx_std_17)
target_include_directories(eye_to_hand_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(eye_to_hand_solver PUBLIC ${OpenCV_LIBRARIES})

add_executable(eye_to_hand_calibrator
  src/eye_to_hand_calibrator.cpp
)
target_compile_features(eye_to_hand_calibrator PUBLIC cxx_std_17)
target_include_directories(eye_to_hand_calibrator PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(eye_to_hand_calibrator eye_to_hand_solver ${OpenCV_LIBRARIES})
ament_target_dependencies(eye_to_hand_calibrator
  cv_bridge
  geometry_msgs
  rclcpp
  sensor_msgs
  std_srvs
  tf2
  tf2_ros
)

add_executable(serial_trajectory_controller
  src/serial_trajectory_controller.cpp
)
target_compile_features(serial_trajectory_controller PUBLIC cxx_std_17)
ament_target_dependencies(serial_trajectory_controller
  control_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
)

add_library(fruit_grasp_attach_plugin SHARED
  src/fruit_grasp_attach_plugin.cpp
)
target_include_directories(fruit_grasp_attach_plugin PRIVATE
  ${GAZEBO_INCLUDE_DIRS}
)
target_link_libraries(fruit_grasp_attach_plugin
  ${GAZEBO_LIBRARIES}
)
ament_target_dependencies(fruit_grasp_attach_plugin
  gazebo_ros
  rclcpp
  std_srvs
)

# Install the Python package directly from this ament_cmake package.
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/jaka_single_arm
)

# Install entry point scripts
install(
  PROGRAMS
    scripts/pick_place_runner
    scripts/fruit_target_node
    scripts/fruit_debug_viewer
    scripts/fruit_debug_window
    scripts/rviz_window_guard
    scripts/hand_eye_static_tf
    scripts/serial_fruit_target_bridge
    scripts/serial_board_emulator
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    eye_to_hand_calibrator
    eye_to_hand_solver
    fruit_grasp_attach_plugin
    serial_trajectory_controller
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include)

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
  # Keep project tests deterministic even when the user site contains pytest
  # plugins that are incompatible with Ubuntu's system pytest.
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_eye_to_hand_solver test/test_eye_to_hand_solver.cpp)
  if(TARGET test_eye_to_hand_solver)
    target_link_libraries(test_eye_to_hand_solver eye_to_hand_solver ${OpenCV_LIBRARIES})
    target_include_directories(test_eye_to_hand_solver PRIVATE ${OpenCV_INCLUDE_DIRS})
  endif()
  set(_pytest_env "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1")
  ament_add_pytest_test(
    test_serial_protocol test/test_serial_protocol.py ENV ${_pytest_env})
  ament_add_pytest_test(
    test_serial_integration test/test_serial_integration.py ENV ${_pytest_env})
  ament_add_pytest_test(
    test_serial_trajectory_safety test/test_serial_trajectory_safety.py
    ENV ${_pytest_env})
  ament_add_pytest_test(
    test_mechanical_limit_consistency test/test_mechanical_limit_consistency.py
    ENV ${_pytest_env})
  ament_add_pytest_test(
    test_target_tracker test/test_target_tracker.py ENV ${_pytest_env})
  ament_add_pytest_test(
    test_object_detector test/test_object_detector.py ENV ${_pytest_env})
  ament_add_pytest_test(
    test_fruit_quality_classifier test/test_fruit_quality_classifier.py
    ENV ${_pytest_env})
  ament_add_pytest_test(
    test_health_fusion test/test_health_fusion.py ENV ${_pytest_env})
  ament_add_pytest_test(
    test_real_mode_guards test/test_real_mode_guards.py ENV ${_pytest_env})
  ament_add_pytest_test(
    test_camera_data_freshness test/test_camera_data_freshness.py ENV ${_pytest_env})
  ament_add_pytest_test(
    test_rgbd_perception test/test_rgbd_perception.py ENV ${_pytest_env})
  ament_add_pytest_test(
    test_debug_viewer_conversion test/test_debug_viewer_conversion.py ENV ${_pytest_env})
  ament_add_pytest_test(
    test_rviz_config test/test_rviz_config.py ENV ${_pytest_env})
  ament_add_pytest_test(
    test_fail_closed test/test_fail_closed.py ENV ${_pytest_env})
  ament_add_pytest_test(
    test_bt_reset test/test_bt_reset.py ENV ${_pytest_env})
endif()

ament_package()
