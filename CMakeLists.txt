cmake_minimum_required(VERSION 3.5)
project(lucia_pressure)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(pressure_serial_node src/pressure_serial_node.cpp)
ament_target_dependencies(pressure_serial_node rclcpp std_msgs)

install(TARGETS pressure_serial_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()