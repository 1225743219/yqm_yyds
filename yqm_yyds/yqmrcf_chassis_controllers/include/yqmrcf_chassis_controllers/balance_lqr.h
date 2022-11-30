/*******************************************************************************
 * Copyright (c) 2022/08/15, Liao LunJia.
 * All rights reserved.
 *******************************************************************************/
#pragma once

#include <effort_controllers/joint_effort_controller.h>
#include <sensor_msgs/Imu.h>
#include <yqmrcf_common/lqr.h>

#include <Eigen/Dense>

#include "yqmrcf_chassis_controllers/chassis_base.h"
namespace yqmrcf_chassis_controllers {

class BalanceLqrController : public ChassisBase {
 public:
  BalanceLqrController() = default;
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

  /** @brief Execute ChassisBase::update(). Set and publish necessary params.
   *
   * @param time The current time.
   * @param period The time passed since the last call to update.
   */
  void update(const ros::Time &time, const ros::Duration &period) override;

 private:
  /** @brief
   *
   * @param a
   * @param b
   * @param q
   * @param r
   */
  void getK(XmlRpc::XmlRpcValue a, XmlRpc::XmlRpcValue b, XmlRpc::XmlRpcValue q, XmlRpc::XmlRpcValue r);
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

  ros::Subscriber base_imu_sub_;
  ros::Publisher state_real_pub_;
  realtime_tools::RealtimeBuffer<sensor_msgs::Imu> base_imu_rt_buffer_;
  sensor_msgs::Imu imu_data_;
  hardware_interface::JointHandle joint_left_, joint_right_;

  // class member about full state feedback controller
  static const int STATE_DIM = 4;
  static const int CONTROL_DIM = 2;
  Eigen::Matrix<double, STATE_DIM, 1> x_{}, x_ref_{};
  Eigen::Matrix<double, CONTROL_DIM, 1> u_{};
  Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> k_{};
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> a_{}, q_{};
  Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> b_{};
  Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> r_{};

  double com_pitch_offset_{};
};

}  // namespace yqmrcf_chassis_controllers
