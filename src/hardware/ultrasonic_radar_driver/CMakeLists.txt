cmake_minimum_required(VERSION 3.8)
project(ultrasonic_radar_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UltrasonicData.msg"
  DEPENDENCIES std_msgs
)

# 获取 typesupport target
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 可执行文件
add_executable(ultrasonic_radar_node
  src/main.cpp
  src/ultrasonic_radar_node.cpp
)

target_include_directories(ultrasonic_radar_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ultrasonic_radar_node
  rclcpp
  std_msgs
)

target_link_libraries(ultrasonic_radar_node "${cpp_typesupport_target}")

# 安装
install(TARGETS
  ultrasonic_radar_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

ament_package()