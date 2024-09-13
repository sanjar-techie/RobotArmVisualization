#include "robot_arm.h"
#include "visualizer.h"
#include <iostream>
#include <vector>
#include <cmath>

int main() {
    // Problem 2a:
    std::vector<double> joint_angles = {0, 0};  
    std::vector<double> link_lengths = {4.0, 2.0};   
    // Problem 2b:         
    // std::vector<double> joint_angles = {-M_PI / 4, -M_PI / 2};  
    // std::vector<double> link_lengths = {4.0, 2.0};           
    // // Problem 2c: 
    // std::vector<double> joint_angles = {M_PI / 8, -2 * M_PI / 3};  
    // std::vector<double> link_lengths = {2.0, 3.0}; 

    RobotArm robot(joint_angles, link_lengths);
    std::vector<JointState> joint_states = robot.calculateForwardKinematics();

    Visualizer vis(1600, 1200);  // Create a 800x600 window
    vis.drawRobot(joint_states);
    vis.saveToFile("robot_arm.png");
    
    // Display the window
    vis.display();

    return 0;
}