cmake_minimum_required(VERSION 3.8)
project(d455_6d_pose)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS calib3d core imgproc)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(pose_core src/pose_core.cpp)
target_compile_features(pose_core PUBLIC cxx_std_17)
target_include_directories(pose_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(pose_core PUBLIC ${OpenCV_LIBRARIES})

add_executable(d455_pose_node src/d455_pose_node.cpp)
target_compile_features(d455_pose_node PUBLIC cxx_std_17)
target_include_directories(d455_pose_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(d455_pose_node pose_core ${OpenCV_LIBRARIES})
ament_target_dependencies(d455_pose_node
  cv_bridge diagnostic_msgs geometry_msgs message_filters rclcpp sensor_msgs
  std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros vision_msgs visualization_msgs)

install(TARGETS pose_core d455_pose_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch docs meshes models
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pose_core test/test_pose_core.cpp)
  if(TARGET test_pose_core)
    target_link_libraries(test_pose_core pose_core ${OpenCV_LIBRARIES})
    target_include_directories(test_pose_core PRIVATE ${OpenCV_INCLUDE_DIRS})
  endif()
endif()

ament_package()
