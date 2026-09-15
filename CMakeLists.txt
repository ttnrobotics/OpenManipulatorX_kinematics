cmake_minimum_required(VERSION 3.10)

project(open_manipulator_x_kinematics)


find_package(Eigen3 REQUIRED)


add_library(
    open_manipulator_x_kinematics
    src/open_manipulator_x_kinematics.cpp
)


target_include_directories(
    open_manipulator_x_kinematics
    PUBLIC
    include
)


target_link_libraries(
    open_manipulator_x_kinematics
    Eigen3::Eigen
)


add_executable(
    test_kinematics
    src/test_kinematics.cpp
)


target_link_libraries(
    test_kinematics
    open_manipulator_x_kinematics
)