/*
 * mobile_base_ros.hpp
 *
 * Created on: Dec 16, 2021 16:13
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#ifndef MOBILE_BASE_ROS_HPP
#define MOBILE_BASE_ROS_HPP

#include <memory>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_broadcaster.h>

#include "wrp_ros/AssistedModeSetCommand.h"
#include "wrp_ros/MotionResetCommand.h"
#include "wrp_ros/MotionCommand.h"
#include "wrp_ros/AccessControl.h"

#include "wrp_sdk/interface/mobile_robot_interface.hpp"

namespace westonrobot {
enum class RobotBaseType { kWeston, kAgilex, kVbot, kUnitreeA1 };

class MobileBaseRos {
 public:
  MobileBaseRos(ros::NodeHandle* nh, const std::string& can,
                const RobotBaseType& robot_base);
  ~MobileBaseRos() = default;

  void SetBaseFrame(std::string base_frame);
  void SetOdomFrame(std::string odom_frame);
  void SetOdomTopicName(std::string odom_topic);

  void SetAutoReconnect(bool enable);

  void Run(double loop_hz);

 private:
  // ros handlers
  ros::NodeHandle* nh_;

  ros::Publisher odom_publisher_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  ros::Publisher system_state_publisher_;
  ros::Publisher motion_state_publisher_;
  ros::Publisher actuator_state_publisher_;

  ros::Publisher ultrasonic_data_publisher_;
  ros::Publisher tof_data_publisher_;

  ros::Subscriber assisted_mode_set_cmd_subscriber_;
  ros::Subscriber motion_cmd_subscriber_;

  ros::ServiceServer access_control_service_;
  ros::ServiceServer light_control_service_;

  // internal control
  std::shared_ptr<MobileRobotInterface> robot_ = nullptr;
  bool auto_reconnect_ = true;

  std::string can_device_ = "can0";
  std::string base_frame_ = "base_link";
  std::string odom_frame_ = "odom";
  std::string odom_topic_name_ = "odom";

  float position_x_ = 0.0;
  float position_y_ = 0.0;
  float theta_ = 0.0;
  ros::Time last_time_ = ros::Time::now();
  double loop_period_ = 0.02;  // in seconds

  void SetupSubscription();
  void SetupService();

  void PublishRobotStateToROS();
  void PublishSensorDataToROS();
  void PublishWheelOdometry();

  void MotionCmdCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void AssistedModeSetCmdCallback(
      const wrp_ros::AssistedModeSetCommand::ConstPtr& msg);

  bool HandleAccessControl(wrp_ros::AccessControl::Request& req,
                           wrp_ros::AccessControl::Response& res);
};
}  // namespace westonrobot

#endif /* MOBILE_BASE_ROS_HPP */
