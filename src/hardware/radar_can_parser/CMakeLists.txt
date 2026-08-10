cmake_minimum_required(VERSION 3.8)
project(radar_can_parser)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

include_directories(include)

add_executable(radar_can_parser_node
  src/can_parser.cpp
  src/radar_parser.cpp
  src/radar_node.cpp
)

ament_target_dependencies(radar_can_parser_node
  rclcpp
  sensor_msgs
  visualization_msgs
  pcl_conversions
)

target_link_libraries(radar_can_parser_node
  ${PCL_LIBRARIES}
)

install(TARGETS radar_can_parser_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()