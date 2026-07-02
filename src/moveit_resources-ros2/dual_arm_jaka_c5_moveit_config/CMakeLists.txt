cmake_minimum_required(VERSION 3.8)
project(dual_arm_jaka_c5_moveit_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-deprecated-declarations -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)

install(
  DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/dual_arm_carry_demo.py
    scripts/dual_arm_massage_demo.py
    scripts/dual_arm_massage_demo_v6.py
    scripts/gazebo_carry_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

# 将 gazebo_ros2_control::GazeboSystem 重新注册为 hardware_interface::SystemInterface
# 插件，以便 ros2_control 的 resource_manager 可以通过 pluginlib 发现并加载它。
# 原插件注册在 gazebo_ros2_control__pluginlib__plugin 命名空间下，
# resource_manager 只在 hardware_interface__pluginlib__plugin 下查找。
pluginlib_export_plugin_description_file(
  hardware_interface
  config/gazebo_ros2_control_hw_interface_plugins.xml
)

# pluginlib 在注册包的 lib/ 目录下查找共享库。
# libgazebo_hardware_plugins.so 属于 gazebo_ros2_control 包，
# 我们需要在安装目录中创建一个符号链接指向它。
install(
  CODE "
    set(lib_dir \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}\")
    file(MAKE_DIRECTORY \${lib_dir})
    execute_process(
      COMMAND ln -sf /opt/ros/humble/lib/libgazebo_hardware_plugins.so
              \${lib_dir}/libgazebo_hardware_plugins.so
      RESULT_VARIABLE _symlink_result
      ERROR_QUIET
    )
    if(NOT _symlink_result EQUAL 0)
      message(WARNING \"Could not create symlink for libgazebo_hardware_plugins.so\")
    endif()
  "
)

ament_package()
