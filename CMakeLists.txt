# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 3.16.3...3.20)
project(robot_self_filter)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()




#common commands for building c++ executables and libraries
ament_auto_add_library(robot_geometric_shapes src/load_mesh.cpp src/shapes.cpp src/bodies.cpp)

#[[
ament_auto_add_library(${PROJECT_NAME} src/self_mask.cpp)
target_link_libraries(${PROJECT_NAME} robot_geometric_shapes assimp)


ament_auto_add_executable(test_filter src/test_filter.cpp)
target_link_libraries(test_filter ${PROJECT_NAME})
target_link_libraries(test_filter robot_geometric_shapes  assimp)

ament_auto_add_executable(self_filter src/self_filter.cpp)
target_link_libraries(self_filter ${PROJECT_NAME})
target_link_libraries(self_filter robot_geometric_shapes  assimp)
]]



ament_auto_package(
)