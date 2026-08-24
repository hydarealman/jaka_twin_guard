cmake_minimum_required(VERSION 3.8)
project(fruit_arm_moveit_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-deprecated-declarations -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/pick_place_demo.py
    scripts/simulated_camera.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
