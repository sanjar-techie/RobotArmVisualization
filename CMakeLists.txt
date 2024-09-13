cmake_minimum_required(VERSION 3.10)
project(RobotArmVisualization)

set(CMAKE_CXX_STANDARD 14)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/eigen-3.4.0)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# sfml for visualization
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

add_executable(robot_arm_visualization 
    src/main.cpp
    src/robot_arm.cpp)

target_link_libraries(robot_arm_visualization sfml-graphics sfml-window sfml-system)