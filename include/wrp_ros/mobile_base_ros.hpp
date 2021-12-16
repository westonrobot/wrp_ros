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

#include "wrp_sdk/mobile_base/interface/mobile_robot_interface.hpp"

namespace westonrobot {
class MobileBaseRos {
 public:
 private:
  // ros handlers
  ros::NodeHandle *nh_;

  ros::Publisher odom_publisher_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  ros::Publisher system_state_publisher_;
  ros::Publisher motion_state_publisher_;
  ros::Publisher actuator_state_publisher_;

  ros::Subscriber motion_cmd_subscriber_;
  ros::Subscriber light_cmd_subscriber_;

  ros::ServiceServer access_control_service_;
  ros::ServiceServer light_state_service_;

  // internal control
  std::shared_ptr<MobileRobotInterface> robot_ = nullptr;

  std::string can_device_ = "can0";
  std::string odom_frame_ = "odom";
  std::string base_frame_ = "base_link";
  std::string odom_topic_name_ = "odom";
};
}  // namespace westonrobot

#endif /* MOBILE_BASE_ROS_HPP */
