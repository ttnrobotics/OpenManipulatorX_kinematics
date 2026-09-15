#include "open_manipulator_x_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>


// Translation matrix
Eigen::Isometry3d OpenManipulatorXKinematics::translation(double x, double y, double z)
{
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    T.translation() = Eigen::Vector3d(x, y, z);

    return T;
}

// Rotation around Y
Eigen::Isometry3d OpenManipulatorXKinematics::rotationY(double angle)
{
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    T.linear() = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix();

    return T;
}

// Rotation around Z
Eigen::Isometry3d OpenManipulatorXKinematics::rotationZ(double angle)
{
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    T.linear() = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    return T;
}

OpenManipulatorXKinematics::FKResult 
OpenManipulatorXKinematics::forwardKinematics(const JointVector &q) const
{
    const double q1 = q(0);
    const double q2 = q(1);
    const double q3 = q(2);
    const double q4 = q(3);

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    // Link 1 -> joint 1
    T *= translation(P1_X, 0.0, 0.0);
    T *= rotationZ(q1);

    // joint 1 -> joint 2
    T *= translation(0.0, 0.0, P2_Z);
    T *= rotationY(q2);

    // joint 2 -> joint 3
    T *= translation(P3_X, 0.0, P3_Z);
    T *= rotationY(q3);

    // joint 3 -> joint 4
    T *= translation(P4_X, 0.0, 0.0);
    T *= rotationY(q4);

    // joint 4 -> end-effector
    T *= translation(TOOL_X, 0.0, 0.0);

    FKResult result;
    result.transform = T;
    result.position = T.translation();
    result.rotation = T.rotation();
    result.yaw = q1;
    result.pitch = q2 + q3 + q4;

    return result;
}