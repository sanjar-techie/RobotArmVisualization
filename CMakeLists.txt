cmake_minimum_required(VERSION 3.10)
project(RobotArmVisualization)

set(CMAKE_CXX_STANDARD 14)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/eigen-3.4.0)

add_executable(robot_arm_visualization src/main.cpp)