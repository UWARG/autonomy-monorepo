cmake_minimum_required(VERSION 3.8)
project(airside_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BuildingWing.msg"
  "msg/Coordinate.msg"
  "msg/LocalizationResult.msg"
  "msg/LocalizedTarget.msg"
  "msg/Plane.msg"
  "msg/ProcessedMap.msg"
  "msg/ReferenceMeasurement.msg"
  "msg/SpatialTarget.msg"
  "msg/Target.msg"
  "msg/WingBoundary.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
