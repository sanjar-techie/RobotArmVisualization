#include "robot_arm.h"
#include <cmath>
#include <stdexcept>
#include <iostream>

RobotArm::RobotArm(const std::vector<double>& joint_angles, const std::vector<double>& link_lengths)
    : joint_angles_(joint_angles), link_lengths_(link_lengths) {
    if (joint_angles_.size() != link_lengths_.size()) {
        throw std::invalid_argument("Number of joint angles must match number of link lengths");
    }
}

Eigen::Matrix3d RobotArm::homogeneousTransform(const std::vector<double>& positions, double theta) {
    Eigen::Matrix3d T;
    T << std::cos(theta), -std::sin(theta), positions[0],
         std::sin(theta),  std::cos(theta), positions[1],
         0,                0,               1;
    return T;
}

std::vector<JointState> RobotArm::calculateForwardKinematics() {
    // g_WE = g_WA * g_AB * g_BE
    std::vector<JointState> joint_states;
    
    // g_WA: 
    Eigen::Matrix3d g_WA = homogeneousTransform({0, 0}, joint_angles_[0]);
    joint_states.push_back({g_WA.block<2,1>(0,2), g_WA.block<2,2>(0,0)});

    // g_WB: 
    Eigen::Matrix3d g_AB = homogeneousTransform({0, link_lengths_[0]}, joint_angles_[1]);
    Eigen::Matrix3d g_WB = g_WA * g_AB;
    joint_states.push_back({g_WB.block<2,1>(0,2), g_WB.block<2,2>(0,0)});

    // g_WE:
    Eigen::Matrix3d g_BE = homogeneousTransform({0, link_lengths_[1]}, 0);
    Eigen::Matrix3d g_WE = g_WB * g_BE;
    std::cout << "End effector position: (" 
              << g_WE(0, 2) << ", " 
              << g_WE(1, 2) << ")" << std::endl;
    std::cout << "End effector orientation: " 
              << std::atan2(g_WE(1, 0), g_WE(0, 0)) << " radians" << std::endl;
    joint_states.push_back({g_WE.block<2,1>(0,2), g_WE.block<2,2>(0,0)});

    return joint_states;
}