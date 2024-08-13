cmake_minimum_required(VERSION 3.8)
project(duro_localizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)

include_directories(
  include
)

add_executable(duro_localizer src/duro_localizer.cpp)
ament_target_dependencies(
  duro_localizer
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  autoware_adapi_v1_msgs
  autoware_vehicle_msgs
)

install(TARGETS
    duro_localizer
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
