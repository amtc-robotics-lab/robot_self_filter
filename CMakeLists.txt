# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 3.16.3...3.20)
project(robot_self_filter)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
Bullet)

#common commands for building c++ executables and libraries
ament_auto_add_library(${PROJECT_NAME} SHARED src/load_mesh.cpp src/shapes.cpp src/bodies.cpp)
target_link_libraries(${PROJECT_NAME} assimp)

# self_filter is built as an rclcpp component so it can be loaded into a composable-node
# container and benefit from intra-process (zero-copy) transport; the register macro also
# generates a standalone "self_filter" executable for running it as its own process.
ament_auto_add_library(self_filter_component SHARED src/self_filter.cpp)
target_link_libraries(self_filter_component ${PROJECT_NAME} assimp)
rclcpp_components_register_node(self_filter_component
  PLUGIN "robot_self_filter::SelfFilterNode"
  EXECUTABLE self_filter
)

ament_auto_add_executable(test_filter src/test_filter.cpp)
target_link_libraries(test_filter ${PROJECT_NAME} assimp)

ament_export_libraries(${PROJECT_NAME} self_filter_component)
ament_export_dependencies(rclcpp rclcpp_components Bullet tf2_bullet tf2_ros sensor_msgs urdf tf2 resource_retriever assimp)
ament_auto_package(
)
