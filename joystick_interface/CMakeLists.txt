cmake_minimum_required(VERSION 3.8)
project(joystick_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(joystick_interface_node
  src/joystick_interface_node.cpp
  src/joystick_interface.cpp
)

ament_target_dependencies(joystick_interface_node
  rclcpp
  geometry_msgs
  sensor_msgs
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  joystick_interface_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()