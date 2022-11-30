/*******************************************************************************
 * Copyright (c) 2022/7/11, Liao LunJia.
 * All rights reserved.
 *******************************************************************************/
#pragma once

#include <control_toolbox/pid.h>
#include <controller_interface/multi_interface_controller.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <yqmrcf_common/filters/filters.h>
#include <yqmrcf_msgs/ChassisCmd.h>

namespace yqmrcf_chassis_controllers {

struct Command {
  geometry_msgs::Twist cmd_vel_;
  yqmrcf_msgs::ChassisCmd cmd_chassis_;
  ros::Time stamp_;
};

class ChassisBase : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface> {
 public:
  ChassisBase() = default;

  /** @brief Get and check params for covariances. Setup odometry realtime publisher + odom message constant fields.
   * init odom tf.
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

  /** @brief Receive real_time command from manual. Execute different action according to current mode. Set
   * necessary params of chassis.
   *
   * Receive real_time command from manual and check whether it is normally, if can not receive command from manual
   * for a while, chassis's velocity will be set zero to avoid out of control. UpdateOdom, Set necessary params such as
   * Acc and vel_tfed.
   *
   * @param time The current time.
   * @param period The time passed since the last call to update.
   */
  void update(const ros::Time &time, const ros::Duration &period) override;

 protected:
  virtual void moveJoint(const ros::Time &time, const ros::Duration &period) = 0;

  virtual geometry_msgs::Twist forwardKinematics() = 0;

  /** @brief Init frame on base_link. Integral vel to pos and angle.
   *
   * @param time The current time.
   * @param period The time passed since the last call to update.
   */
  void updateOdom(const ros::Time &time, const ros::Duration &period);

  /** @brief Write current command from yqmrcf_msgs::ChassisCmd.
   *
   * @param msg This message contains various state parameter settings for basic chassis control
   */
  void cmdChassisCallback(const yqmrcf_msgs::ChassisCmd::ConstPtr &msg);

  /** @brief Write current command from  geometry_msgs::Twist.
   *
   * @param msg This expresses velocity in free space broken into its linear and angular parts.
   */
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);

  hardware_interface::EffortJointInterface *effort_joint_interface_{};
  std::vector<hardware_interface::JointHandle> joint_handles_{};

  double wheel_base_{}, wheel_track_{}, wheel_radius_{}, publish_rate_{}, timeout_{};
  bool enable_odom_tf_ = false;
  bool publish_odom_tf_ = false;
  yqmrcf_filters::RampFilter<double> *ramp_x_{}, *ramp_y_{}, *ramp_w_{};

  ros::Time last_publish_time_;
  geometry_msgs::TransformStamped odom2base_{};
  geometry_msgs::Vector3 vel_cmd_{};  // x, y

  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber cmd_chassis_sub_;
  Command cmd_struct_;
  realtime_tools::RealtimeBuffer<Command> cmd_rt_buffer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener *tf_listener_{};
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};
}  // namespace yqmrcf_chassis_controllers
