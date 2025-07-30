cmake_minimum_required(VERSION 3.8)
project(ros_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(MSG_SRC_DIR ${PROJECT_SOURCE_DIR}/msg)

file(GLOB_RECURSE MSG_SRC_ABSOLUTE ${MSG_SRC_DIR}/*.msg)

set(REQUIRED_PACKAGES 
  ament_cmake
  rosidl_default_generators
  std_msgs
  sensor_msgs
)

# find dependencies
foreach(PKG ${REQUIRED_PACKAGES})
  find_package(${PKG} REQUIRED)
endforeach()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(MSG_SRC "")
foreach(msg_file ${MSG_SRC_ABSOLUTE})
  file(RELATIVE_PATH relative_msg_file ${PROJECT_SOURCE_DIR} ${msg_file})
  list(APPEND MSG_SRC ${relative_msg_file})
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME} ${MSG_SRC} DEPENDENCIES std_msgs sensor_msgs)

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

ament_package()
