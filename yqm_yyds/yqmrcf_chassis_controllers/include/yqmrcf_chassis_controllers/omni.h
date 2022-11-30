/*******************************************************************************
 * Copyright (c) 2022/08/30, Liao LunJia.
 * All rights reserved.
 *******************************************************************************/
#pragma once
#include <effort_controllers/joint_velocity_controller.h>

#include "yqmrcf_chassis_controllers/chassis_base.h"
namespace yqmrcf_chassis_controllers {

class OmniController : public ChassisBase {
 public:
  OmniController() = default;

  bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

 private:
  void moveJoint(const ros::Time &time, const ros::Duration &period) override;
  geometry_msgs::Twist forwardKinematics() override;

  double chassis_radius_{};
  effort_controllers::JointVelocityController ctrl_lf_, ctrl_rf_, ctrl_lb_, ctrl_rb_;
};

}  // namespace yqmrcf_chassis_controllers
