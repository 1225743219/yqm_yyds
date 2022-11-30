/*******************************************************************************
 * Copyright (c) 2022/7/11, Liao LunJia.
 * All rights reserved.
 *******************************************************************************/
#pragma once

#include <effort_controllers/joint_effort_controller.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <sensor_msgs/Imu.h>

#include "yqmrcf_chassis_controllers/chassis_base.h"
#include "yqmrcf_common/filters/filters.h"

namespace yqmrcf_chassis_controllers {

class BalancePidController : public ChassisBase {
 public:
  BalancePidController() = default;
  /** @brief Execute ChassisBase::init. Get necessary handles.
   *
   * Execute ChassisBase::init. Init nh_left, nh_right handles so that we can controls two wheels conveniently.
   *
   * @param robot_hw The robot hardware abstraction.
   * @param root_nh A NodeHandle in the root of the controller manager namespace. This is where the ROS interfaces are
   * setup (publishers, subscribers, services).
   * @param controller_nh A NodeHandle in the namespace of the controller. This is where the controller-specific
   * configuration resides.
   * @return True if initialization was successful and the controller
   * is ready to be started.
   */
  bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

 private:
  /** @brief Calculate correct command and set it to two wheels.
   *
   * @param time The current time.
   * @param period The time passed since the last call to update.
   */
  void moveJoint(const ros::Time &time, const ros::Duration &period) override;
  /** @brief Calculate current linear_x and angular_z according to current velocity.
   *
   * @return Calculated vel_data included linear_x and angular_z.
   */
  geometry_msgs::Twist forwardKinematics() override;
  /** @brief Write imu data from sensor_msgs::Imu.
   *
   * @param msg Imu data in base_link coordinate axis .
   */
  void baseImuCallback(const sensor_msgs::Imu::ConstPtr &msg);

  double keepVelocity(const ros::Duration &period);
  double keepVertical(double angle_des, const ros::Duration &period);
  double keepTurn(const ros::Duration &period);

  hardware_interface::JointHandle joint_left_, joint_right_;
  control_toolbox::Pid pid_velocity_, pid_vertical_, pid_turn_;
  ros::Subscriber base_imu_sub_;
  realtime_tools::RealtimeBuffer<sensor_msgs::Imu> base_imu_rt_buffer_;

  yqmrcf_filters::LowPassFilter<double> *velocity_filter_{};
  double com_pitch_offset_{};
};
}  // namespace yqmrcf_chassis_controllers
