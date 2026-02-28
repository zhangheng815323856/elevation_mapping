cmake_minimum_required(VERSION 3.8)
project(elevation_mapping_ros2)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(elevation_mapping_node src/elevation_mapping_node.cpp)
ament_target_dependencies(elevation_mapping_node
  rclcpp
  sensor_msgs
  nav_msgs
  std_msgs
  visualization_msgs
)

target_compile_features(elevation_mapping_node PUBLIC cxx_std_17)

install(TARGETS elevation_mapping_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
