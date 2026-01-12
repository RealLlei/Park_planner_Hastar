cmake_minimum_required(VERSION 3.8)
project(hybrid_a_star_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

# -------- Hybrid A* core library (你的算法) --------
add_library(hybrid_a_star_core
  src/hybrid_a_star_core.cpp
)

ament_target_dependencies(hybrid_a_star_core
  rclcpp
)

# -------- ROS2 Node --------
add_executable(hybrid_a_star_node
  src/hybrid_a_star_node.cpp
)

target_link_libraries(hybrid_a_star_node
  hybrid_a_star_core
)

ament_target_dependencies(hybrid_a_star_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  hybrid_a_star_core
  hybrid_a_star_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
