cmake_minimum_required(VERSION 3.8)
project(mcsim_odom_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/odom_demo.py
  scripts/odom_demo_with_waves.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
