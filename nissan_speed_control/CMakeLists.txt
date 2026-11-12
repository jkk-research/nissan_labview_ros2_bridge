cmake_minimum_required(VERSION 3.8)
project(nissan_speed_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nissan_bridge_msgs REQUIRED)

add_executable(speed_control src/speed_control_node.cpp)
target_compile_features(speed_control PUBLIC cxx_std_17)
ament_target_dependencies(speed_control rclcpp geometry_msgs std_msgs nissan_bridge_msgs)

install(TARGETS speed_control
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
