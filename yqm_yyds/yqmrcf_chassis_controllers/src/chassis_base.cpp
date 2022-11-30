/*******************************************************************************
 * Copyright (c) 2022/7/11, Liao LunJia.
 * All rights reserved.
 *******************************************************************************/

#include "yqmrcf_chassis_controllers/chassis_base.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace yqmrcf_chassis_controllers {
bool ChassisBase::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh,
                       ros::NodeHandle &controller_nh) {
  if (!controller_nh.getParam("publish_rate", publish_rate_) || !controller_nh.getParam("timeout", timeout_)) {
    ROS_ERROR("Some chassis params doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  controller_nh.getParam("wheel_radius", wheel_radius_);
  controller_nh.getParam("wheel_track", wheel_track_);
  controller_nh.getParam("wheel_base", wheel_base_);
  controller_nh.param("enable_odom_tf", enable_odom_tf_, true);

  XmlRpc::XmlRpcValue twist_cov_list;
  controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);

  effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();

  // Setup odometry realtime publisher + odom message constant fields
  odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(root_nh, "odom", 100));
  odom_pub_->msg_.header.frame_id = "odom";
  odom_pub_->msg_.child_frame_id = "base_link";
  odom_pub_->msg_.twist.covariance = {static_cast<double>(twist_cov_list[0]), 0., 0., 0., 0., 0., 0.,
                                      static_cast<double>(twist_cov_list[1]), 0., 0., 0., 0., 0., 0.,
                                      static_cast<double>(twist_cov_list[2]), 0., 0., 0., 0., 0., 0.,
                                      static_cast<double>(twist_cov_list[3]), 0., 0., 0., 0., 0., 0.,
                                      static_cast<double>(twist_cov_list[4]), 0., 0., 0., 0., 0., 0.,
                                      static_cast<double>(twist_cov_list[5])};

  ramp_x_ = new yqmrcf_filters::RampFilter<double>(0, 0.001);
  ramp_y_ = new yqmrcf_filters::RampFilter<double>(0, 0.001);
  ramp_w_ = new yqmrcf_filters::RampFilter<double>(0, 0.001);

  // init odom tf
  tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
  if (enable_odom_tf_) {
    odom2base_.header.frame_id = "odom";
    odom2base_.header.stamp = ros::Time::now();
    odom2base_.child_frame_id = "base_link";
    odom2base_.transform.rotation.w = 1;
    tf_broadcaster_.sendTransform(odom2base_);
  }

  cmd_chassis_sub_ =
      controller_nh.subscribe<yqmrcf_msgs::ChassisCmd>("command", 1, &ChassisBase::cmdChassisCallback, this);
  cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &ChassisBase::cmdVelCallback, this);

  return true;
}

void ChassisBase::update(const ros::Time &time, const ros::Duration &period) {
  yqmrcf_msgs::ChassisCmd cmd_chassis = cmd_rt_buffer_.readFromRT()->cmd_chassis_;
  geometry_msgs::Twist cmd_vel = cmd_rt_buffer_.readFromRT()->cmd_vel_;

  if ((time - cmd_rt_buffer_.readFromRT()->stamp_).toSec() > timeout_) {
    vel_cmd_.x = 0.;
    vel_cmd_.y = 0.;
    vel_cmd_.z = 0.;
  } else {
    ramp_x_->setAcc(cmd_chassis.accel.linear.x);
    ramp_y_->setAcc(cmd_chassis.accel.linear.y);
    ramp_x_->input(cmd_vel.linear.x);
    ramp_y_->input(cmd_vel.linear.y);
    vel_cmd_.x = ramp_x_->output();
    vel_cmd_.y = ramp_y_->output();
    vel_cmd_.z = cmd_vel.angular.z;
  }

  updateOdom(time, period);

  ramp_w_->setAcc(cmd_chassis.accel.angular.z);
  ramp_w_->input(vel_cmd_.z);
  vel_cmd_.z = ramp_w_->output();

  moveJoint(time, period);
}  // ChassisBase::update

void ChassisBase::updateOdom(const ros::Time &time, const ros::Duration &period) {
  geometry_msgs::Twist vel_base = forwardKinematics();  // on base_link frame
  if (enable_odom_tf_) {
    geometry_msgs::Vector3 linear_vel_odom, angular_vel_odom;
    try {
      odom2base_ = tf_buffer_.lookupTransform("odom", "base_link", ros::Time(0));
    } catch (tf2::TransformException &ex) {
      tf_broadcaster_.sendTransform(odom2base_);  // TODO: For some reason, the sendTransform in init sometime not work?
      ROS_WARN("%s", ex.what());
      return;
    }
    odom2base_.header.stamp = time;
    // integral vel to pos and angle
    tf2::doTransform(vel_base.linear, linear_vel_odom, odom2base_);
    tf2::doTransform(vel_base.angular, angular_vel_odom, odom2base_);
    odom2base_.transform.translation.x += linear_vel_odom.x * period.toSec();
    odom2base_.transform.translation.y += linear_vel_odom.y * period.toSec();
    odom2base_.transform.translation.z += linear_vel_odom.z * period.toSec();
    double length =
        std::sqrt(std::pow(angular_vel_odom.x, 2) + std::pow(angular_vel_odom.y, 2) + std::pow(angular_vel_odom.z, 2));
    if (length > 0.001) {  // avoid nan quat
      tf2::Quaternion odom2base_quat, trans_quat;
      tf2::fromMsg(odom2base_.transform.rotation, odom2base_quat);
      trans_quat.setRotation(
          tf2::Vector3(angular_vel_odom.x / length, angular_vel_odom.y / length, angular_vel_odom.z / length),
          length * period.toSec());
      odom2base_quat = trans_quat * odom2base_quat;
      odom2base_quat.normalize();
      odom2base_.transform.rotation = tf2::toMsg(odom2base_quat);
    }
    tf_broadcaster_.sendTransform(odom2base_);
  }

  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time) {
    if (odom_pub_->trylock()) {
      odom_pub_->msg_.header.stamp = time;
      odom_pub_->msg_.twist.twist.linear.x = vel_base.linear.x;
      odom_pub_->msg_.twist.twist.linear.y = vel_base.linear.y;
      odom_pub_->msg_.twist.twist.angular.z = vel_base.angular.z;
      odom_pub_->unlockAndPublish();
    }
    last_publish_time_ = time;
  }
}  // ChassisBase::updateOdom

void ChassisBase::cmdChassisCallback(const yqmrcf_msgs::ChassisCmd::ConstPtr &msg) {
  cmd_struct_.cmd_chassis_ = *msg;
  cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
}

void ChassisBase::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
  cmd_struct_.cmd_vel_ = *msg;
  cmd_struct_.stamp_ = ros::Time::now();
  cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
}

}  // namespace yqmrcf_chassis_controllers
