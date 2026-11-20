cmake_minimum_required(VERSION 3.8)
project(global_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(airside_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(octomap REQUIRED)

# Library (algorithm + helpers)
add_library(global_planner_lib
  src/theta_star.cpp
)
target_include_directories(global_planner_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OCTOMAP_INCLUDE_DIRS}
)
ament_target_dependencies(global_planner_lib
  rclcpp
  geometry_msgs
  nav_msgs
  octomap_msgs
  mavros_msgs
  std_msgs
  airside_interfaces
  octomap
)
target_link_libraries(global_planner_lib ${OCTOMAP_LIBRARIES})

# Executable (ROS 2 node)
add_executable(global_planner_node src/planner.cpp)
target_include_directories(global_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OCTOMAP_INCLUDE_DIRS}
)
target_link_libraries(global_planner_node global_planner_lib)
ament_target_dependencies(global_planner_node
  rclcpp
  geometry_msgs
  nav_msgs
  octomap_msgs
  mavros_msgs
  std_msgs
  airside_interfaces
  octomap
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_theta_star test/test_theta_star.cpp)
  target_include_directories(test_theta_star PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${OCTOMAP_INCLUDE_DIRS}
  )
  target_link_libraries(test_theta_star global_planner_lib ${OCTOMAP_LIBRARIES})
  ament_target_dependencies(test_theta_star
    rclcpp
    geometry_msgs
    nav_msgs
    octomap_msgs
    mavros_msgs
    std_msgs
    airside_interfaces
    octomap
  )
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Installation
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS global_planner_lib global_planner_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
endif()

# Export targets and dependencies
ament_export_include_directories(include)
ament_export_libraries(global_planner_lib)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  airside_interfaces
  geometry_msgs
  nav_msgs
  octomap_msgs
  mavros_msgs
  std_msgs
  octomap
)

ament_package()
