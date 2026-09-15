#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>


class OpenManipulatorXKinematics
{
public:
    using JointVector = Eigen::Matrix<double, 4, 1>;

    struct FKResult
    {
        Eigen::Isometry3d transform;
        Eigen::Vector3d position;
        Eigen::Matrix3d rotation;
        double roll;
        double yaw;
        double pitch;
    };


    OpenManipulatorXKinematics() = default;


    // Forward Kinematics
    FKResult forwardKinematics(const JointVector &q) const;


    // Calculate all valid IK solutions
    std::vector<JointVector> inverseKinematics(
        const Eigen::Vector3d &target_position,
        double target_pitch) const;


    // Select IK solution nearest to current joint state
    bool inverseKinematics(
        const Eigen::Vector3d &target_position,
        double target_pitch,
        const JointVector &current_q,
        JointVector &solution) const;


    bool withinJointLimits(const JointVector &q) const;


private:

    // --------------------------------------------------
    // OpenManipulator-X geometry [m]
    // --------------------------------------------------

    static constexpr double P1_X = 0.012;

    static constexpr double P2_Z = 0.0595;

    static constexpr double P3_X = 0.024;
    static constexpr double P3_Z = 0.128;

    static constexpr double P4_X = 0.124;

    static constexpr double TOOL_X = 0.126;


    static constexpr double PI = 3.14159265358979323846;


    // --------------------------------------------------
    // Helper functions
    // --------------------------------------------------

    static Eigen::Isometry3d translation(double x, double y, double z);

    static Eigen::Isometry3d rotationY(double angle);

    static Eigen::Isometry3d rotationZ(double angle);

    static double normalizeAngle(double angle);

    static double angleDifference(double a, double b);
};