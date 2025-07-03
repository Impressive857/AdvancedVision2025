cmake_minimum_required(VERSION 3.8)
project(vision_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(EXEC_SRC_DIR ${PROJECT_SOURCE_DIR}/src)
set(EXEC_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(EXEC_LIBRARY_DIR ${PROJECT_SOURCE_DIR}/lib)

set(EXEC_DEPEND rclcpp OpenCV cv_bridge yaml-cpp sensor_msgs ros_msg image_transport)
set(ALL_EXEC ${PROJECT_NAME}_node)

file(GLOB_RECURSE EXEC_SOURCES "${EXEC_SRC_DIR}/*.cpp")

set(REQUIRED_PACKAGES 
  ament_cmake_auto
  rclcpp
  OpenCV
  cv_bridge
  yaml-cpp
  std_msgs
  ros_msg
  image_transport
)

# find dependencies
foreach(PKG ${REQUIRED_PACKAGES})
  find_package(${PKG} REQUIRED)
endforeach()

set(CMAKE_AUTOMOC ON)

# find dependencies
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/../vision_utils)
include_directories(${EXEC_INCLUDE_DIR}/${PROJECT_NAME})

ament_auto_add_executable(${PROJECT_NAME}_node ${EXEC_SOURCES} ${EXEC_INCLUDE_DIR}/${PROJECT_NAME}/vision_system.hpp ${EXEC_INCLUDE_DIR}/${PROJECT_NAME}/system_node.hpp)
ament_target_dependencies(${PROJECT_NAME}_node ${EXEC_DEPEND})

target_link_libraries(${PROJECT_NAME}_node
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  yaml-cpp
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
