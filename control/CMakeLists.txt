cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(Boost REQUIRED)

add_executable(joycon_node src/joycon_node.cpp
                           src/joycon.cpp)
target_include_directories(
  joycon_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  joycon_node
  rclcpp
  sensor_msgs
  geometry_msgs
  diagnostic_updater
)

install(TARGETS
  joycon_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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