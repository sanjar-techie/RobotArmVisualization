#include "robot_arm.h"
#include <cmath>
#include <stdexcept>
#include <iostream>

RobotArm::RobotArm(const std::vector<double>& joint_angles, 
                   const std::vector<double>& link_lengths,
                   bool zero_config_along_x)
    : joint_angles_(joint_angles), 
      link_lengths_(link_lengths),
      zero_config_along_x_(zero_config_along_x) {
    if (joint_angles_.size() != link_lengths_.size()) {
        throw std::invalid_argument("Number of joint angles must match number of link lengths");
    }
}

Eigen::Matrix3d RobotArm::homogeneousTransform(const std::vector<double>& positions, double theta) {
    Eigen::Matrix3d T;
    if (zero_config_along_x_) {
        // Zero configuration along x-axis
        T << std::cos(theta), -std::sin(theta), positions[0],
             std::sin(theta),  std::cos(theta), positions[1],
             0,                0,               1;
    } else {
        // Zero configuration along y-axis
        T << std::cos(theta), -std::sin(theta), positions[1],
             std::sin(theta),  std::cos(theta), positions[0],
             0,                0,               1;
    }
    return T;
}

std::vector<JointState> RobotArm::calculateForwardKinematics() {
    std::vector<JointState> joint_states;
    
    // Initial position
    std::vector<double> init_pos = {0, 0};
    Eigen::Matrix3d g_WA = homogeneousTransform(init_pos, joint_angles_[0]);
    joint_states.push_back({g_WA.block<2,1>(0,2), g_WA.block<2,2>(0,0)});

    // First link
    std::vector<double> link1_pos;
    if (zero_config_along_x_) {
        link1_pos = {link_lengths_[0], 0};
    } else {
        link1_pos = {0, link_lengths_[0]};
    }
    Eigen::Matrix3d g_AB = homogeneousTransform(link1_pos, joint_angles_[1]);
    Eigen::Matrix3d g_WB = g_WA * g_AB;
    joint_states.push_back({g_WB.block<2,1>(0,2), g_WB.block<2,2>(0,0)});

    // End effector
    std::vector<double> link2_pos;
    if (zero_config_along_x_) {
        link2_pos = {link_lengths_[1], 0};
    } else {
        link2_pos = {0, link_lengths_[1]};
    }
    Eigen::Matrix3d g_BE = homogeneousTransform(link2_pos, 0);
    Eigen::Matrix3d g_WE = g_WB * g_BE;
    
    std::cout << "End effector position: (" 
              << g_WE(0, 2) << ", " 
              << g_WE(1, 2) << ")" << std::endl;
    std::cout << "End effector orientation: " 
              << std::atan2(g_WE(1, 0), g_WE(0, 0)) << " radians" << std::endl;
    
    joint_states.push_back({g_WE.block<2,1>(0,2), g_WE.block<2,2>(0,0)});

    return joint_states;
}