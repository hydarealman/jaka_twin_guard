cmake_minimum_required(VERSION 3.8)
project(fruit_picking_arm)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(control_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS aruco calib3d core imgcodecs imgproc)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

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

add_library(calibration_target_detector
  src/calibration_target_detector.cpp
)
target_compile_features(calibration_target_detector PUBLIC cxx_std_17)
target_include_directories(calibration_target_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(calibration_target_detector PUBLIC eye_to_hand_solver ${OpenCV_LIBRARIES})

add_library(kinematic_calibration
  src/kinematic_calibration.cpp
)
target_compile_features(kinematic_calibration PUBLIC cxx_std_17)
target_include_directories(kinematic_calibration PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(kinematic_calibration PUBLIC
  eye_to_hand_solver
  Ceres::ceres
  Eigen3::Eigen
  yaml-cpp
  ${OpenCV_LIBRARIES}
)

add_executable(kinematic_calibrator
  src/kinematic_calibrator.cpp
)
target_compile_features(kinematic_calibrator PUBLIC cxx_std_17)
target_include_directories(kinematic_calibrator PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(kinematic_calibrator
  calibration_target_detector
  kinematic_calibration
  yaml-cpp
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(kinematic_calibrator
  cv_bridge
  rclcpp
  sensor_msgs
  std_srvs
  tf2
  tf2_ros
)

add_library(serial_protocol
  src/serial_protocol.cpp
)
target_compile_features(serial_protocol PUBLIC cxx_std_17)
target_include_directories(serial_protocol PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(serial_trajectory_controller
  src/serial_trajectory_controller.cpp
)
target_compile_features(serial_trajectory_controller PUBLIC cxx_std_17)
target_link_libraries(serial_trajectory_controller serial_protocol)
target_include_directories(serial_trajectory_controller PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(serial_trajectory_controller
  control_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
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
  PACKAGE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/fruit_picking_arm
)

# Install entry point scripts
install(
  PROGRAMS
    scripts/pick_place_runner
    scripts/fruit_target_node
    scripts/fruit_rviz_goal_bridge
    scripts/fruit_debug_viewer
    scripts/fruit_debug_window
    scripts/rviz_window_guard
    scripts/hand_eye_static_tf
    scripts/hand_eye_calibration_dashboard
    scripts/serial_fruit_target_bridge
    scripts/serial_board_emulator
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    eye_to_hand_calibrator
    eye_to_hand_solver
    calibration_target_detector
    kinematic_calibration
    kinematic_calibrator
    serial_protocol
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
  DIRECTORY fruit_picking_arm/behavior/trees
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
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_kinematic_calibration.cpp")
    ament_add_gtest(test_kinematic_calibration test/test_kinematic_calibration.cpp)
    if(TARGET test_kinematic_calibration)
      target_link_libraries(test_kinematic_calibration
        kinematic_calibration
        Ceres::ceres
        Eigen3::Eigen
        yaml-cpp
        ${OpenCV_LIBRARIES}
      )
      target_include_directories(test_kinematic_calibration PRIVATE
        ${OpenCV_INCLUDE_DIRS}
      )
    endif()
  else()
    message(WARNING
      "Skipping test_kinematic_calibration: test source is not present")
  endif()
  ament_add_gtest(test_serial_protocol_cpp test/test_serial_protocol.cpp)
  if(TARGET test_serial_protocol_cpp)
    target_link_libraries(test_serial_protocol_cpp serial_protocol)
    target_include_directories(test_serial_protocol_cpp PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
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
    test_frame_contract test/test_frame_contract.py ENV ${_pytest_env})
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
    test_real_gripper_contract test/test_real_gripper_contract.py ENV ${_pytest_env})
  ament_add_pytest_test(
    test_solidworks_urdf_pipeline test/test_solidworks_urdf_pipeline.py
    ENV ${_pytest_env})
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
  ament_add_pytest_test(
    test_bin_geometry test/test_bin_geometry.py ENV ${_pytest_env})
  ament_add_pytest_test(
    test_rgbd_table_estimator test/test_rgbd_table_estimator.py ENV ${_pytest_env})
  ament_add_pytest_test(
    test_top_down_pose test/test_top_down_pose.py ENV ${_pytest_env})
  ament_add_pytest_test(
    test_production_auto_contract test/test_production_auto_contract.py
    ENV ${_pytest_env})
  ament_add_pytest_test(
    test_motion_stop_races test/test_motion_stop_races.py ENV ${_pytest_env})
  ament_add_pytest_test(
    test_source_progress test/test_source_progress.py ENV ${_pytest_env})
endif()

ament_package()
