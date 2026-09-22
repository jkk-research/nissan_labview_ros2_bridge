cmake_minimum_required(VERSION 3.8)
project(nissan_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nissan_bridge_msgs REQUIRED)
find_package(Threads REQUIRED)

add_executable(nissan_bridge_node src/nissan_bridge_node.cpp)
ament_target_dependencies(nissan_bridge_node rclcpp nissan_bridge_msgs)

add_executable(udp_dummy_node src/udp_dummy_node.cpp)
ament_target_dependencies(udp_dummy_node rclcpp)
target_link_libraries(udp_dummy_node Threads::Threads)

install(TARGETS
  nissan_bridge_node
  udp_dummy_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
