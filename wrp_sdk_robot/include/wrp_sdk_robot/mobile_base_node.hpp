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
#include "wrp_sdk_msgs/RcState.h"

#include "wrp_sdk/interface/mobile_robot_interface.hpp"

namespace westonrobot {
class MobileBaseNode {
 public:
  MobileBaseNode(ros::NodeHandle* nh);
  ~MobileBaseNode() = default;

  void Run(double loop_hz);

 private:
  enum class RobotVariant {
    // AglieX
    kAgilexScoutV2 = 0,
    kAgilexScoutMini,
    kAgilexScoutMiniOmni,
    kAgilexRanger,
    kAgilexRangerMiniV1,
    kAgilexRangerMiniV2,
    kAgilexTracer,
    kAgilexTracerMini,
    kAgilexHunter,
    kAgilexHunterSE,
    kAgilexBunker,

    // WestonRobot
    kWRScout,
    kWRVBot,

    // BangBang
    kBangBangRobooterX,

    kNumOfVariants
  };
  // ----- ROS Node Parameters -----
  int robot_type_ = 0;                   /**< Robot Type/Variant*/
  std::string can_device_ = "can0";      /**< CAN device to use*/
  std::string base_frame_ = "base_link"; /**< Robot base frame name*/
  std::string odom_frame_ = "odom";      /**< Odometry frame name*/
  bool publish_odom_tf_ = true; /**< If publish odom_frame->base_frame tf*/
  bool auto_reconnect_ = true;  /**< If auto request for control*/
  // ----- Internal Variables -----
  double loop_period_;
  ros::NodeHandle* nh_;
  RobotVariant robot_variant_;
  std::shared_ptr<MobileRobotInterface> robot_ = nullptr;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  // ----- Published Messages-----
  std::mutex odom_mutex_;
  nav_msgs::Odometry odom_msg_;
  ros::Time last_odom_time_;
  // ----- Subscribers & Publishers & Services-----
  ros::Subscriber motion_cmd_subscriber_;

  ros::Publisher system_state_publisher_;
  ros::Publisher motion_state_publisher_;
  ros::Publisher actuator_state_publisher_;
  ros::Publisher battery_state_publisher_;
  ros::Publisher rc_state_publisher_;
  ros::Publisher odom_publisher_;
  
  ros::ServiceServer access_control_service_;
  ros::ServiceServer light_control_service_;
  ros::ServiceServer assisted_mode_control_service_;
  ros::ServiceServer motion_reset_service_;

  // ----- Callbacks -----
  void MotionCmdCallback(const geometry_msgs::Twist::ConstPtr& msg);

  bool AccessControlCallback(wrp_sdk_msgs::AccessControl::Request& req,
                           wrp_sdk_msgs::AccessControl::Response& res);
  bool AssistedModeControlCallback(
      wrp_sdk_msgs::AssistedModeControl::Request& req,
      wrp_sdk_msgs::AssistedModeControl::Response& res);
  bool LightControlCallback(wrp_sdk_msgs::LightControl::Request& req,
                          wrp_sdk_msgs::LightControl::Response& res);
  bool MotionResetCallback(wrp_sdk_msgs::MotionReset::Request& req,
                         wrp_sdk_msgs::MotionReset::Response& res);

  bool ReadParameters();
  bool InitInternalState();
  bool SetupHardware();
  bool SetupInterfaces();

  void PublishSystemState();
  void PublishBatteryState();
  void PublishActuatorState();
  void PublishRcState();
  void PublishOdometry();
  void UpdateOdometry(geometry_msgs::Twist twist);
};

}  // namespace westonrobot

#endif /* WRP_SDK_ROBOT_MOBILE_BASE_NODE_HPP */
