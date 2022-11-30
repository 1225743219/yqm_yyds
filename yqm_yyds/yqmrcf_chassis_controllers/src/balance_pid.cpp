/*******************************************************************************
 * Copyright (c) 2022/7/11, Liao LunJia.
 * All rights reserved.
 *******************************************************************************/

#include "yqmrcf_chassis_controllers/balance_pid.h"

#include <tf2/utils.h>

#include <pluginlib/class_list_macros.hpp>

#include "yqmrcf_common/math_utilities.h"
#include "yqmrcf_common/ori_tool.h"
namespace yqmrcf_chassis_controllers {
bool BalancePidController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh,
                                ros::NodeHandle &controller_nh) {
  ChassisBase::init(robot_hw, root_nh, controller_nh);
  ros::NodeHandle nh_left = ros::NodeHandle(controller_nh, "left");
  ros::NodeHandle nh_right = ros::NodeHandle(controller_nh, "right");
  joint_left_ =
      effort_joint_interface_->getHandle(controller_nh.param("joint_left_name", std::string("left_wheel_joint")));
  joint_right_ =
      effort_joint_interface_->getHandle(controller_nh.param("joint_right_name", std::string("right_wheel_joint")));

  if (controller_nh.hasParam("pid_velocity")) {
    if (!pid_velocity_.init(ros::NodeHandle(controller_nh, "pid_velocity"))) return false;
  }

  if (controller_nh.hasParam("pid_vertical")) {
    if (!pid_vertical_.init(ros::NodeHandle(controller_nh, "pid_vertical"))) return false;
  }

  if (controller_nh.hasParam("pid_turn")) {
    if (!pid_turn_.init(ros::NodeHandle(controller_nh, "pid_turn"))) return false;
  }

  com_pitch_offset_ = controller_nh.param("com_pitch_offset", 0.0);

  //  for power limit, but balance not need now
  joint_handles_.push_back(joint_left_);
  joint_handles_.push_back(joint_right_);
  base_imu_sub_ = root_nh.subscribe<sensor_msgs::Imu>("base_imu", 1, &BalancePidController::baseImuCallback, this);

  velocity_filter_ = new yqmrcf_filters::LowPassFilter<double>(0.3);
  return true;
}

void BalancePidController::moveJoint(const ros::Time &time, const ros::Duration &period) {
  double angle_des = keepVelocity(period);
  double vertical_cmd = keepVertical(angle_des, period);
  double turn_effort_cmd = keepTurn(period);

  double left_cmd = vertical_cmd - turn_effort_cmd;
  double right_cmd = vertical_cmd + turn_effort_cmd;

  // TODO effort limit

  joint_left_.setCommand(left_cmd);
  joint_right_.setCommand(right_cmd);
}

// Ref: http://wiki.ros.org/diff_drive_controller
// Ref: https://en.wikipedia.org/wiki/Differential_wheeled_robot
geometry_msgs::Twist BalancePidController::forwardKinematics() {
  geometry_msgs::Twist vel_data;
  double left_velocity = joint_left_.getVelocity();
  double right_velocity = joint_right_.getVelocity();

  vel_data.linear.x = ((left_velocity + right_velocity) / 2) * wheel_radius_;
//  vel_data.angular.z = (right_velocity - left_velocity) * wheel_radius_ / wheel_track_;
  vel_data.angular.z = base_imu_rt_buffer_.readFromRT()->angular_velocity.z;
  return vel_data;
}

void BalancePidController::baseImuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  base_imu_rt_buffer_.writeFromNonRT(*msg);
}

double BalancePidController::keepVelocity(const ros::Duration &period) {
  geometry_msgs::Twist vel_base = forwardKinematics();
  double velocity_error = vel_cmd_.x - vel_base.linear.x;
  velocity_filter_->input(velocity_error);
  velocity_error = velocity_filter_->output();
  pid_velocity_.computeCommand(velocity_error, period);
  double angle_des = minAbs(pid_velocity_.getCurrentCmd(), 0.174);
  return angle_des + com_pitch_offset_;
}

double BalancePidController::keepVertical(double angle_des, const ros::Duration &period) {
  geometry_msgs::Vector3 quat_base;
  quatToRPY(base_imu_rt_buffer_.readFromRT()->orientation, quat_base.x, quat_base.y, quat_base.z);
  double vertical_error = quat_base.y - angle_des;
  double gory_y = base_imu_rt_buffer_.readFromRT()->angular_velocity.y;
  pid_vertical_.computeCommand(vertical_error, gory_y, period);
  return pid_vertical_.getCurrentCmd();
}

double BalancePidController::keepTurn(const ros::Duration &period) {
  double turn_error = vel_cmd_.z - base_imu_rt_buffer_.readFromRT()->angular_velocity.z;
  pid_turn_.computeCommand(turn_error, period);
  return minAbs(pid_turn_.getCurrentCmd(), 10.0);
}

}  // namespace yqmrcf_chassis_controllers

PLUGINLIB_EXPORT_CLASS(yqmrcf_chassis_controllers::BalancePidController, controller_interface::ControllerBase)
