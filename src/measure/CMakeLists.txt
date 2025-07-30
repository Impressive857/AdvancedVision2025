cmake_minimum_required(VERSION 3.8)
project(measure)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(EXEC_DEPEND rclcpp OpenCV cv_bridge yaml-cpp sensor_msgs ros_msgs image_transport vision_utils)
set(ALL_EXEC ${PROJECT_NAME}_node)

set(EXEC_SRC_DIR ${PROJECT_SOURCE_DIR}/src)
set(EXEC_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(EXEC_LIBRARY_DIR ${PROJECT_SOURCE_DIR}/lib)

set(ASCEND_CL_INCLUDE_DIR /usr/local/Ascend/ascend-toolkit/latest/include)
set(ASCEND_CL_LIB_DIR /usr/local/Ascend/ascend-toolkit/latest/lib64)

set(REQUIRED_PACKAGES 
  ament_cmake_auto
  rclcpp
  OpenCV
  cv_bridge
  yaml-cpp
  std_msgs
  ros_msgs
  image_transport
  vision_utils
)

file(GLOB_RECURSE EXEC_SOURCES "${EXEC_SRC_DIR}/*.cpp")

# find dependencies
foreach(PKG ${REQUIRED_PACKAGES})
  find_package(${PKG} REQUIRED)
endforeach()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/../vision_utils/utils/include)
include_directories(${PROJECT_SOURCE_DIR}/../vision_utils/model/include)
include_directories(${ASCEND_CL_INCLUDE_DIR})
include_directories(${EXEC_INCLUDE_DIR}/${PROJECT_NAME})

ament_auto_add_executable(${PROJECT_NAME}_node ${EXEC_SOURCES})
ament_target_dependencies(${PROJECT_NAME}_node ${EXEC_DEPEND})

target_link_libraries(${PROJECT_NAME}_node
  yaml-cpp
  ${ASCEND_CL_LIB_DIR}/libascendcl.so
)

install(TARGETS ${ALL_EXEC} DESTINATION lib/${PROJECT_NAME})

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
