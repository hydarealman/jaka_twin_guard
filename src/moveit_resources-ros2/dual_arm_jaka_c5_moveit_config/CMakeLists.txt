cmake_minimum_required(VERSION 3.8)
project(dual_arm_jaka_c5_moveit_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-deprecated-declarations -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/dual_arm_carry_demo.py
    scripts/gazebo_carry_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
