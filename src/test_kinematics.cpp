#include "open_manipulator_x_kinematics.hpp"

#include <iostream>


int main()
{
    OpenManipulatorXKinematics kinematics;


    // --------------------------------------------------
    // Example joint state
    // --------------------------------------------------

    OpenManipulatorXKinematics::JointVector q;

    q <<
        0.3,
        -0.3,
        0.7,
        -0.2;


    // --------------------------------------------------
    // Forward Kinematics
    // --------------------------------------------------

    auto fk = kinematics.forwardKinematics(q);


    std::cout
        << "===== Forward Kinematics ====="
        << std::endl;


    std::cout
        << "Joint angles:\n"
        << q
        << "\n\n";


    std::cout
        << "End-effector position:\n"
        << fk.position
        << "\n\n";


    std::cout
        << "Rotation matrix:\n"
        << fk.rotation
        << "\n\n";


    std::cout
        << "Transform:\n"
        << fk.transform.matrix()
        << "\n\n";


    std::cout << "Roll  : " << fk.roll << " rad\n";
    std::cout << "Pitch : " << fk.pitch << " rad\n";
    std::cout << "Yaw   : " << fk.yaw << " rad\n";


    // --------------------------------------------------
    // Inverse Kinematics
    // --------------------------------------------------

    OpenManipulatorXKinematics::JointVector q_ik;


    bool success =
        kinematics.inverseKinematics(
            fk.position,
            fk.pitch,
            q,
            q_ik);


    if (!success)
    {
        std::cout
            << "IK failed."
            << std::endl;

        return 1;
    }


    std::cout
        << "\n===== Inverse Kinematics ====="
        << std::endl;


    std::cout
        << "IK solution:\n"
        << q_ik
        << std::endl;


    // --------------------------------------------------
    // Check solution using FK again
    // --------------------------------------------------

    auto fk_check =
        kinematics.forwardKinematics(q_ik);


    std::cout
        << "\nIK -> FK position:\n"
        << fk_check.position
        << std::endl;


    std::cout
        << "\nPosition error:\n"
        << fk.position -
           fk_check.position
        << std::endl;


    return 0;
}