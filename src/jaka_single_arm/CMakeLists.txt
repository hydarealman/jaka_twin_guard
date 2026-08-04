cmake_minimum_required(VERSION 3.8)
project(jaka_single_arm)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS aruco calib3d core imgproc)
find_package(rclcpp REQUIRED)
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
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS eye_to_hand_calibrator eye_to_hand_solver fruit_grasp_attach_plugin
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
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_eye_to_hand_solver test/test_eye_to_hand_solver.cpp)
  if(TARGET test_eye_to_hand_solver)
    target_link_libraries(test_eye_to_hand_solver eye_to_hand_solver ${OpenCV_LIBRARIES})
    target_include_directories(test_eye_to_hand_solver PRIVATE ${OpenCV_INCLUDE_DIRS})
  endif()
  ament_add_pytest_test(test_serial_protocol test/test_serial_protocol.py)
  ament_add_pytest_test(test_serial_integration test/test_serial_integration.py)
  ament_add_pytest_test(test_serial_trajectory_safety test/test_serial_trajectory_safety.py)
  ament_add_pytest_test(test_mechanical_limit_consistency test/test_mechanical_limit_consistency.py)
  ament_add_pytest_test(test_target_tracker test/test_target_tracker.py)
  ament_add_pytest_test(test_object_detector test/test_object_detector.py)
  ament_add_pytest_test(test_bt_reset test/test_bt_reset.py)
endif()

ament_package()
