cmake_minimum_required(VERSION 3.13)
project(utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(REQUIRED_PACKAGES 
  ament_cmake_auto
  rclcpp
  yaml-cpp
  std_msgs
  ros_msgs
  OpenCV
  cv_bridge
)

set(LIB_DEPEND rclcpp yaml-cpp sensor_msgs ros_msgs std_msgs OpenCV cv_bridge)

set(ASCEND_CL_INCLUDE_DIR /usr/local/Ascend/ascend-toolkit/latest/include)

foreach(PKG ${REQUIRED_PACKAGES})
  find_package(${PKG} REQUIRED)
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(${ASCEND_CL_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)

ament_package()