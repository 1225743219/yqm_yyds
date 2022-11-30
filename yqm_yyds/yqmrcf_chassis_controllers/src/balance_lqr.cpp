/*******************************************************************************
 * Copyright (c) 2022/08/15, Liao LunJia.
 * All rights reserved.
 *******************************************************************************/

#include "yqmrcf_chassis_controllers/balance_lqr.h"

#include <yqmrcf_msgs/BalanceState.h>

#include <pluginlib/class_list_macros.hpp>

#include "yqmrcf_common/ori_tool.h"

namespace yqmrcf_chassis_controllers {
bool BalanceLqrController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh,
                                ros::NodeHandle &controller_nh) {
  ChassisBase::init(robot_hw, root_nh, controller_nh);

  joint_left_ =
      effort_joint_interface_->getHandle(controller_nh.param("joint_left_name", std::string("left_wheel_joint")));
  joint_right_ =
      effort_joint_interface_->getHandle(controller_nh.param("joint_right_name", std::string("right_wheel_joint")));
  joint_handles_.push_back(joint_left_);
  joint_handles_.push_back(joint_right_);

  com_pitch_offset_ = controller_nh.param("com_pitch_offset", 0.0);

  XmlRpc::XmlRpcValue a, b, q, r;
  controller_nh.getParam("a", a);
  controller_nh.getParam("b", b);
  controller_nh.getParam("q", q);
  controller_nh.getParam("r", r);
  getK(a, b, q, r);

  state_real_pub_ = root_nh.advertise<yqmrcf_msgs::BalanceState>("state_real", 100);
  base_imu_sub_ = root_nh.subscribe<sensor_msgs::Imu>("base_imu", 1, &BalanceLqrController::baseImuCallback, this);

  return true;
}

void BalanceLqrController::update(const ros::Time &time, const ros::Duration &period) {
  ChassisBase::update(time, period);
  yqmrcf_msgs::BalanceState state_real_msg;
  state_real_msg.header.stamp = time;
  state_real_msg.x_dot = x_(0);
  state_real_msg.delta_dot = x_(1);
  state_real_msg.theta = x_(2);
  state_real_msg.theta_dot = x_(3);
  state_real_msg.c_l = joint_left_.getEffort();
  state_real_msg.c_r = joint_right_.getEffort();
  state_real_pub_.publish(state_real_msg);
}

void BalanceLqrController::moveJoint(const ros::Time &time, const ros::Duration &period) {
  x_ref_(0) = vel_cmd_.x;
  x_ref_(1) = vel_cmd_.z;

  imu_data_ = *base_imu_rt_buffer_.readFromRT();
  double roll{}, pitch{}, yaw{};
  quatToRPY(imu_data_.orientation, roll, pitch, yaw);
  x_ << forwardKinematics().linear.x, forwardKinematics().angular.z, pitch + com_pitch_offset_,
      imu_data_.angular_velocity.y;
  u_ = k_ * (x_ref_ - x_);

  joint_left_.setCommand(u_(0));
  joint_right_.setCommand(u_(1));
}

geometry_msgs::Twist BalanceLqrController::forwardKinematics() {
  geometry_msgs::Twist vel_data;
  double joint_left_velocity = joint_left_.getVelocity();
  double joint_right_velocity = joint_right_.getVelocity();
  vel_data.linear.x = ((joint_left_velocity + joint_right_velocity) / 2) * wheel_radius_;
  //  vel_data.angular.z = (-joint_left_velocity + joint_right_velocity) *
  //  wheel_radius_ / wheel_track_;
  vel_data.angular.z = base_imu_rt_buffer_.readFromRT()->angular_velocity.z;
  return vel_data;
}

void BalanceLqrController::baseImuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  base_imu_rt_buffer_.writeFromNonRT(*msg);
}

void BalanceLqrController::getK(XmlRpc::XmlRpcValue a, XmlRpc::XmlRpcValue b, XmlRpc::XmlRpcValue q,
                                XmlRpc::XmlRpcValue r) {
  // Check a and q
  ROS_ASSERT(a.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(q.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(a.size() == STATE_DIM);
  ROS_ASSERT(q.size() == STATE_DIM);
  for (int i = 0; i < STATE_DIM; ++i) {
    ROS_ASSERT(a[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(q[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(a[i].size() == STATE_DIM);
    ROS_ASSERT(q[i].size() == STATE_DIM);
    for (int j = 0; j < STATE_DIM; ++j) {
      ROS_ASSERT(a[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
                 a[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt);
      ROS_ASSERT(q[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
                 q[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt);
      if (a[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble)
        a_(i, j) = static_cast<double>(a[i][j]);
      else if (a[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt)
        a_(i, j) = static_cast<int>(a[i][j]);
      if (q[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble)
        q_(i, j) = static_cast<double>(q[i][j]);
      else if (q[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt)
        q_(i, j) = static_cast<int>(q[i][j]);
    }
  }
  // Check b
  ROS_ASSERT(b.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(b.size() == STATE_DIM);
  for (int i = 0; i < STATE_DIM; ++i) {
    ROS_ASSERT(b[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(b[i].size() == CONTROL_DIM);
    for (int j = 0; j < CONTROL_DIM; ++j) {
      ROS_ASSERT(b[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
                 b[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt);
      if (b[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble)
        b_(i, j) = static_cast<double>(b[i][j]);
      else if (b[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt)
        b_(i, j) = static_cast<int>(b[i][j]);
    }
  }
  // Check r
  ROS_ASSERT(r.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(r.size() == CONTROL_DIM);
  for (int i = 0; i < CONTROL_DIM; ++i) {
    ROS_ASSERT(r[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(r[i].size() == CONTROL_DIM);
    for (int j = 0; j < CONTROL_DIM; ++j) {
      ROS_ASSERT(r[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
                 r[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt);
      if (r[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble)
        r_(i, j) = static_cast<double>(r[i][j]);
      else if (r[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt)
        r_(i, j) = static_cast<int>(r[i][j]);
    }
  }

  Lqr<double> lqr(a_, b_, q_, r_);
  lqr.computeK();
  k_ = lqr.getK();
}

}  // namespace yqmrcf_chassis_controllers

PLUGINLIB_EXPORT_CLASS(yqmrcf_chassis_controllers::BalanceLqrController, controller_interface::ControllerBase)
