/*******************************************************************************
 * Copyright (c) 2022/7/25, Liao LunJia.
 * All rights reserved.
 *******************************************************************************/
#pragma once

#include <effort_controllers/joint_velocity_controller.h>

#include "yqmrcf_chassis_controllers/chassis_base.h"

namespace yqmrcf_chassis_controllers {
class MecanumController : public ChassisBase {
 public:
  MecanumController() = default;
  /** @brief Execute ChassisBase::init. Get necessary handles.
   *
   * Execute ChassisBase::init. Init lf, lb, rf, rb handles so that we can controls four wheels conveniently.
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
  /** @brief Calculate correct command and set it to four wheels.
   *
   * @param time The current time.
   * @param period The time passed since the last call to update.
   */
  void moveJoint(const ros::Time &time, const ros::Duration &period) override;
  /** @brief Calculate current linear_x, linear_y and angular_z according to current velocity.
   *
   * @return Calculated vel_data included linear_x, linear_y and angular_z.
   */
  geometry_msgs::Twist forwardKinematics() override;

  effort_controllers::JointVelocityController ctrl_lf_, ctrl_rf_, ctrl_lb_, ctrl_rb_;
};
}  // namespace yqmrcf_chassis_controllers
