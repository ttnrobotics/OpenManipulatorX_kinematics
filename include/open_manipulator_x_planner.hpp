#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class OpenManipulatorXPlanner
{
public:
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

  OpenManipulatorXPlanner(
    const rclcpp::Node::SharedPtr & node,
    const std::string & planning_group = "arm")
  : node_(node),
    move_group_(node_, planning_group)
  {
    move_group_.setPlanningTime(5.0);
    move_group_.setMaxVelocityScalingFactor(0.3);
    move_group_.setMaxAccelerationScalingFactor(0.3);
  }

  bool moveJ(const std::vector<double> & joint_positions)
  {
    move_group_.setJointValueTarget(joint_positions);

    MoveGroupInterface::Plan plan;
    bool success =
      static_cast<bool>(move_group_.plan(plan));

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "MoveJ planning failed");
      return false;
    }

    auto result = move_group_.execute(plan);

    return result == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool moveL(
    const geometry_msgs::msg::Pose & target_pose,
    double eef_step = 0.005,
    double jump_threshold = 0.0)
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;

    geometry_msgs::msg::Pose start_pose = move_group_.getCurrentPose().pose;

    waypoints.push_back(start_pose);
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;

    double fraction =
      move_group_.computeCartesianPath(
        waypoints,
        eef_step,
        jump_threshold,
        trajectory);

    if (fraction < 0.95) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "MoveL Cartesian path failed. Fraction: %.2f",
        fraction);
      return false;
    }

    MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    auto result = move_group_.execute(plan);

    return result == moveit::core::MoveItErrorCode::SUCCESS;
  }

  void setSpeed(double velocity_scale, double acceleration_scale)
  {
    move_group_.setMaxVelocityScalingFactor(velocity_scale);
    move_group_.setMaxAccelerationScalingFactor(acceleration_scale);
  }

private:
  rclcpp::Node::SharedPtr node_;
  MoveGroupInterface move_group_;
};