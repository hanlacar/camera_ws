cmake_minimum_required(VERSION 3.8)
project(camera_bringup)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_d456_serial.py")
    ament_add_pytest_test(test_d456_serial test/test_d456_serial.py)
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_two_view_rqt.py")
    ament_add_pytest_test(test_two_view_rqt test/test_two_view_rqt.py)
  endif()
endif()

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
