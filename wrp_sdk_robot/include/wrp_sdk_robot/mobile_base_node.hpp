/**
 * @file mobile_base_node.hpp
 * @brief
 * @date 17-05-2024
 *
 * @copyright Copyright (c) 2024 Weston Robot Pte. Ltd.
 */
#ifndef WRP_SDK_ROBOT_MOBILE_BASE_NODE_HPP
#define WRP_SDK_ROBOT_MOBILE_BASE_NODE_HPP

#include <memory>
#include <string>
#include <mutex>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/BatteryState.h>
#include <tf2_ros/transform_broadcaster.h>

#include "wrp_sdk_msgs/MotionCommand.h"
#include "wrp_sdk_msgs/AccessControl.h"
#include "wrp_sdk_msgs/AssistedModeControl.h"
#include "wrp_sdk_msgs/LightControl.h"
#include "wrp_sdk_msgs/MotionReset.h"

#include "wrp_sdk/interface/mobile_robot_interface.hpp"

namespace westonrobot {
class MobileBaseNode {
 public:
  MobileBaseNode(ros::NodeHandle* nh);
  ~MobileBaseNode() = default;

  void Run(double loop_hz);

 private:
  // ----- ROS Node Parameters -----
  bool auto_reconnect_ = true;
  std::string can_device_ = "can0";
  std::string base_frame_ = "base_link";
  std::string robot_type_ = "weston";
  std::string odom_frame_ = "odom";
  std::string motion_type_ = "skid_steer";
  bool publish_odom_tf_ = true;
  // ----- Internal Variables -----
  ros::NodeHandle* nh_;
  std::shared_ptr<MobileRobotInterface> robot_ = nullptr;

  float position_x_ = 0.0;
  float position_y_ = 0.0;
  float theta_ = 0.0;
  ros::Time last_time_ = ros::Time::now();
  double loop_period_ = 0.02;  // in seconds

  // ----- Published Messages-----
  // ----- Subscribers & Publishers & Services-----
  ros::Publisher odom_publisher_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  ros::Publisher system_state_publisher_;
  ros::Publisher motion_state_publisher_;
  ros::Publisher actuator_state_publisher_;
  ros::Publisher battery_state_publisher_;
  ros::Publisher ultrasonic_data_publisher_;
  ros::Publisher tof_data_publisher_;
  ros::Subscriber motion_cmd_subscriber_;

  ros::ServiceServer access_control_service_;
  ros::ServiceServer light_control_service_;
  ros::ServiceServer assisted_mode_control_service_;
  ros::ServiceServer motion_reset_service_;

  // ----- Callbacks -----
  void MotionCmdCallback(const geometry_msgs::Twist::ConstPtr& msg);

  bool HandleAccessControl(wrp_sdk_msgs::AccessControl::Request& req,
                           wrp_sdk_msgs::AccessControl::Response& res);
  bool HandleAssistedModeControl(
      wrp_sdk_msgs::AssistedModeControl::Request& req,
      wrp_sdk_msgs::AssistedModeControl::Response& res);
  bool HandleLightControl(wrp_sdk_msgs::LightControl::Request& req,
                          wrp_sdk_msgs::LightControl::Response& res);
  bool HandleMotionReset(wrp_sdk_msgs::MotionReset::Request& req,
                         wrp_sdk_msgs::MotionReset::Response& res);

  bool ReadParameters();
  bool InitInternalState();
  bool SetupHardware();
  bool SetupInterfaces();

  void PublishRobotState();
  void PublishSensorData();
  void PublishWheelOdometry();
  nav_msgs::Odometry CalculateOdometry(geometry_msgs::Twist robot_twist);
};

}  // namespace westonrobot

#endif /* WRP_SDK_ROBOT_MOBILE_BASE_NODE_HPP */
