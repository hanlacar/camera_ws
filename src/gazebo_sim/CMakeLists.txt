cmake_minimum_required(VERSION 3.8)
project(gazebo_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Python module (cmd_conversion + nodes)
ament_python_install_package(${PROJECT_NAME})

# Executable node entry points
install(PROGRAMS
  gazebo_sim/cmd_vel_adapter_node.py
  gazebo_sim/imu_splitter_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Resources
install(DIRECTORY launch config urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
