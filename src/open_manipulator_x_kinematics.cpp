#include "open_manipulator_x_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>


// Translation matrix

Eigen::Isometry3d
OpenManipulatorXKinematics::translation(double x, double y, double z)
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
    T = T * translation(P1_X, 0.0, 0.0);
    T = T * rotationZ(q1);

    // joint 1 -> joint 2
    T = T * translation(0.0, 0.0, P2_Z);
    T = T * rotationY(q2);

    // joint 2 -> joint 3
    T = T * translation(P3_X, 0.0, P3_Z);
    T = T * rotationY(q3);

    // joint 3 -> joint 4
    T = T * translation(P4_X, 0.0, 0.0);
    T = T * rotationY(q4);

    // joint 4 -> end-effector
    T = T * translation(TOOL_X, 0.0, 0.0);

    FKResult result;
    result.transform = T;
    result.position = T.translation();
    result.rotation = T.rotation();

    const Eigen::Matrix3d &R = result.rotation;

    // ZYX Euler convention
    result.roll = std::atan2(
        R(2, 1),
        R(2, 2));

    result.pitch = std::atan2(
        -R(2, 0),
        std::sqrt(
            R(2, 1) * R(2, 1) +
            R(2, 2) * R(2, 2)));

    result.yaw = std::atan2(
        R(1, 0),
        R(0, 0));

    return result;
}

std::vector<OpenManipulatorXKinematics::JointVector>
OpenManipulatorXKinematics::inverseKinematics(
    const Eigen::Vector3d &target_position,
    double target_pitch) const
{
    std::vector<JointVector> solutions;


    // --------------------------------------------------
    // Target position
    // --------------------------------------------------
    const double x = target_position.x();
    const double y = target_position.y();
    const double z = target_position.z();


    // --------------------------------------------------
    // Joint 1
    // --------------------------------------------------
    const double dx = x - P1_X;
    const double dy = y;
    const double q1 = std::atan2(dy, dx);
    const double radial = std::sqrt(dx * dx + dy * dy);

    // --------------------------------------------------
    // Remove joint2 height
    // --------------------------------------------------
    const double z_relative = z - P2_Z;

    // --------------------------------------------------
    // Remove tool offset
    // --------------------------------------------------
    const double wrist_r = radial - TOOL_X * std::cos(target_pitch);
    const double wrist_z = z_relative + TOOL_X * std::sin(target_pitch);

    // --------------------------------------------------
    // Equivalent planar link lengths
    // --------------------------------------------------
    const double L1 = std::sqrt(P3_X * P3_X + P3_Z * P3_Z);
    const double L2 = P4_X;
    const double beta = std::atan2(P3_Z, P3_X);


    // --------------------------------------------------
    // Law of cosines
    // --------------------------------------------------

    double D =
        (wrist_r * wrist_r + wrist_z * wrist_z - L1 * L1 - L2 * L2)
        /
        (2.0 * L1 * L2);


    // Target outside workspace
    const double epsilon = 1e-9;

    if (D > 1.0 + epsilon || D < -1.0 - epsilon)
    {
        return solutions;
    }

    D = std::clamp(D, -1.0, 1.0);


    // --------------------------------------------------
    // Two elbow configurations
    // --------------------------------------------------

    for (double sign : {-1.0, 1.0})
    {
        const double sin_delta = sign * std::sqrt(std::max(0.0, 1.0 - D * D));

        const double delta = std::atan2(sin_delta, D);

        // First planar link angle
        const double theta =
            std::atan2(wrist_z, wrist_r) - std::atan2(L2 * std::sin(delta), L1 + L2 * std::cos(delta));


        double q2 = beta - theta;

        double q3 = -delta - beta;

        double q4 = target_pitch - q2 - q3;

        JointVector q;

        q <<
            normalizeAngle(q1),
            normalizeAngle(q2),
            normalizeAngle(q3),
            normalizeAngle(q4);


        // Only return physically allowed solutions
        if (withinJointLimits(q))
        {
            solutions.push_back(q);
        }
    }


    return solutions;
}




bool OpenManipulatorXKinematics::inverseKinematics(
    const Eigen::Vector3d &target_position,
    double target_pitch,
    const JointVector &current_q,
    JointVector &solution) const
{
    const auto solutions = inverseKinematics(target_position, target_pitch);


    if (solutions.empty())
    {
        return false;
    }

    double best_cost = std::numeric_limits<double>::max();


    for (const auto &candidate : solutions)
    {
        double cost = 0.0;


        for (int i = 0; i < 4; ++i)
        {
            const double error = angleDifference(candidate(i), current_q(i));

            cost += error * error;
        }

        if (cost < best_cost)
        {
            best_cost = cost;
            solution = candidate;
        }
    }


    return true;
}

bool OpenManipulatorXKinematics::withinJointLimits(const JointVector &q) const
{
    const double q1_min = -PI;
    const double q1_max = PI;

    const double q2_min = -1.5;
    const double q2_max = 1.5;

    const double q3_min = -1.5;
    const double q3_max = 1.4;

    const double q4_min = -1.7;
    const double q4_max = 1.97;


    if (q(0) < q1_min || q(0) > q1_max)
    {
        return false;
    }

    if (q(1) < q2_min || q(1) > q2_max)
    {
        return false;
    }

    if (q(2) < q3_min || q(2) > q3_max)
    {
        return false;
    }

    if (q(3) < q4_min || q(3) > q4_max)
    {
        return false;
    }

    return true;
}

double OpenManipulatorXKinematics::normalizeAngle(double angle)
{
    while (angle > PI)
    {
        angle -= 2.0 * PI;
    }

    while (angle < -PI)
    {
        angle += 2.0 * PI;
    }

    return angle;
}


double OpenManipulatorXKinematics::angleDifference(double a, double b)
{
    return normalizeAngle(a - b);
}