cmake_minimum_required(VERSION 3.13)
project(vision_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(POLICY CMP0079)
  cmake_policy(SET CMP0079 NEW)
endif()

# find dependencies
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

add_subdirectory(${PROJECT_SOURCE_DIR}/model)
add_subdirectory(${PROJECT_SOURCE_DIR}/utils)

file(GLOB_RECURSE CONFIG_FILES ${PROJECT_SOURCE_DIR}/config/*)
file(GLOB_RECURSE MODEL_FILES ${PROJECT_SOURCE_DIR}/model/*)
file(GLOB_RECURSE UTILS_FILES ${PROJECT_SOURCE_DIR}/utils/*)
install(FILES ${CONFIG_FILES} DESTINATION share/${PROJECT_NAME}/config)
install(FILES ${MODEL_FILES} DESTINATION share/${PROJECT_NAME}/model)
install(FILES ${UTILS_FILES} DESTINATION share/${PROJECT_NAME}/utils)

ament_package()