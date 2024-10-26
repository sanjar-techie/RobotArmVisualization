#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include <Eigen/Dense>
#include <vector>

struct JointState {
    Eigen::Vector2d position;
    Eigen::Matrix2d orientation;
};

class RobotArm {
public:
    RobotArm(const std::vector<double>& joint_angles, const std::vector<double>& link_lengths, bool zero_config_along_x = true);
    
    std::vector<JointState> calculateForwardKinematics();

private:
    std::vector<double> joint_angles_;
    std::vector<double> link_lengths_;
    bool zero_config_along_x_;

    Eigen::Matrix3d homogeneousTransform(const std::vector<double>& positions, double theta);
};

#endif // ROBOT_ARM_H