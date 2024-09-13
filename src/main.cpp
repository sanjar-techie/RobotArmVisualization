#include "robot_arm.h"
#include "visualizer.h"
#include <iostream>
#include <vector>
#include <cmath>

int main() {
    std::vector<double> joint_angles = {-M_PI / 4, -M_PI / 2};  // Example angles
    std::vector<double> link_lengths = {2.0, 1.5};             // Example link lengths

    RobotArm robot(joint_angles, link_lengths);
    std::vector<JointState> joint_states = robot.calculateForwardKinematics();

    Visualizer vis(1600, 1200);  // Create a 800x600 window
    vis.drawRobot(joint_states);
    vis.saveToFile("robot_arm.png");
    
    // Display the window
    vis.display();

    return 0;
}