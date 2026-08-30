cmake_minimum_required(VERSION 3.8)
project(gazebo_car_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf worlds config models
  DESTINATION share/${PROJECT_NAME}
)

install(CODE
  "file(REMOVE
    \"${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/ackermann_cmd_mux.py\"
    \"${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/V1_lane_detector_node.py\"
    \"${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/cmd_vel_arbiter\")"
)

function(install_program_if_exists script_path executable_name)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${script_path}")
    install(
      PROGRAMS ${script_path}
      DESTINATION lib/${PROJECT_NAME}
      RENAME ${executable_name}
    )
  else()
    message(STATUS "Skipping missing optional program: ${script_path}")
  endif()
endfunction()

install_program_if_exists(scripts/lane_perception.py lane_perception)
install_program_if_exists(scripts/lane_control.py lane_control)
install_program_if_exists(scripts/vehicle_cmd_adapter.py vehicle_cmd_adapter)
install_program_if_exists(scripts/white_line_follow_node.py white_line_follow_node)
install_program_if_exists(scripts/rear_lidar_roi_node.py rear_lidar_roi_node)
install_program_if_exists(scripts/rear_lidar_marker_node.py rear_lidar_marker_node)
install_program_if_exists(scripts/race_b_blinking_obstacle.py race_b_blinking_obstacle)
install_program_if_exists(scripts/race_b_spawn_m2wr_once.py race_b_spawn_m2wr_once)

ament_package()
