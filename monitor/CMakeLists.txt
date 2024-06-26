cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(Boost REQUIRED)

add_executable(monitor_node src/monitor_node.cpp
                           src/monitor.cpp)
target_include_directories(
  monitor_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  monitor_node
  rclcpp
  std_msgs
  geometry_msgs
  diagnostic_updater
)

install(TARGETS
  monitor_node
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