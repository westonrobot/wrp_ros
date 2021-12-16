/*
 * mobile_base_messenger.hpp
 *
 * Created on: Jul 25, 2021 16:38
 * Description:
 *
 * Copyright (c) 2021 Weston Robot
 */

#ifndef MOBILE_BASE_MESSENGER_HPP
#define MOBILE_BASE_MESSENGER_HPP

#include <string>
#include <memory>

#include <nav_msgs/odometry.hpp>
#include <geometry_msgs/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "wrp_ros/light_command.hpp"
#include "wrp_ros/motion_state.hpp"
#include "wrp_ros/range_data.hpp"
#include "wrp_ros/bumper_state.hpp"

#include "wrp_ros/request_control.hpp"
#include "wrp_ros/renounce_control.hpp"
#include "wrp_ros/actuator_state_check.hpp"
#include "wrp_ros/light_state_check.hpp"
#include "wrp_ros/rc_state_check.hpp"
#include "wrp_ros/system_state_check.hpp"

#include "wrp_sdk/mobile_base/westonrobot/mobile_base.hpp"
#include "wrp_sdk/mobile_base/agilex/scout_base_v1_adapter.hpp"
#include "wrp_sdk/mobile_base/agilex/agilex_base_v2_adapter.hpp"

namespace westonrobot {
class MobileBaseROSMessenger {
 public:
  MobileBaseROSMessenger(std::string node_name,
                         std::shared_ptr<MobileRobotInterface> robot = nullptr);

  bool IsInSimMode() const { return simulated_robot_; }
  int GetSimControlRate() const { return sim_control_rate_; }
  std::string GetCanDeveice() const { return can_device_; }

  void SetOdomFrame(std::string odom_frame);
  void SetBaseFrame(std::string base_frame);
  void SetOdomTopicName(std::string odom_topic);

  void SetupSubscription();
  void SetupServices();
  bool GetControlToken();

  void PublishStateToROS();
  void PublishSimStateToROS(double linear, double angular);

 private:
  std::shared_ptr<MobileRobotInterface> robot_ = nullptr;

  std::string can_device_ = "can0";
  bool simulated_robot_ = false;
  int sim_control_rate_ = 50;

  std::string odom_frame_ = "odom";
  std::string base_frame_ = "base_link";
  std::string odom_topic_name_ = "odom";

  float linear_speed_tuning_factor_;
  float angular_speed_tuning_factor_;

  // publisher
  //   rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  //   rclcpp::Publisher<wrp_ros::msg::MotionState>::SharedPtr
  //       motion_state_publisher_;
  //   rclcpp::Publisher<wrp_ros::msg::RangeData>::SharedPtr
  //       ultrasonic_data_publisher_;
  //   rclcpp::Publisher<wrp_ros::msg::RangeData>::SharedPtr
  //   tof_data_publisher_;

  //   // subscriber
  //   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
  //       motion_cmd_subscription_;
  //   rclcpp::Subscription<wrp_ros::msg::LightCommand>::SharedPtr
  //       light_cmd_subscription_;

  //   // servicer
  //   rclcpp::Service<wrp_ros::srv::RequestControl>::SharedPtr
  //       request_control_service_;
  //   rclcpp::Service<wrp_ros::srv::RenounceControl>::SharedPtr
  //       renounce_control_service_;
  //   rclcpp::Service<wrp_ros::srv::SystemStateCheck>::SharedPtr
  //       system_check_service_;
  //   rclcpp::Service<wrp_ros::srv::RcStateCheck>::SharedPtr rc_check_service_;
  //   rclcpp::Service<wrp_ros::srv::ActuatorStateCheck>::SharedPtr
  //       actuator_check_service_;
  //   rclcpp::Service<wrp_ros::srv::LightStateCheck>::SharedPtr
  //       light_check_service_;

  //   std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::mutex twist_mutex_;
  geometry_msgs::msg::Twist current_twist_;

  // speed variables
  double linear_speed_ = 0.0;
  double angular_speed_ = 0.0;
  double position_x_ = 0.0;
  double position_y_ = 0.0;
  double theta_ = 0.0;

  rclcpp::Time last_time_;
  rclcpp::Time current_time_;

  void LoadParameters();
  void EnableSimMode();

  void TwistCmdCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void LightCmdCallback(const wrp_ros::LightCommand::ConstPtr& msg);
  void PublishOdometryToROS(double linear, double angular, double dt);

  // callback function
  //   void RequestControlCallback(
  //       const std::shared_ptr<wrp_ros::srv::RequestControl::Request> request,
  //       std::shared_ptr<wrp_ros::srv::RequestControl::Response> response);
  //   void RenounceControlCallback(
  //       const std::shared_ptr<wrp_ros::srv::RenounceControl::Request>
  //       request, std::shared_ptr<wrp_ros::srv::RenounceControl::Response>
  //       response);
  //   void SystemStateCallback(
  //       const std::shared_ptr<wrp_ros::srv::SystemStateCheck::Request>
  //       request, std::shared_ptr<wrp_ros::srv::SystemStateCheck::Response>
  //       response);
  //   void RcStateCheckCallback(
  //       const std::shared_ptr<wrp_ros::srv::RcStateCheck::Request> request,
  //       std::shared_ptr<wrp_ros::srv::RcStateCheck::Response> response);
  //   void ActuatorStateCheckCallback(
  //       const std::shared_ptr<wrp_ros::srv::ActuatorStateCheck::Request>
  //           request,
  //       std::shared_ptr<wrp_ros::srv::ActuatorStateCheck::Response>
  //       response);
  //   void LightStateCheckCallback(
  //       const std::shared_ptr<wrp_ros::srv::LightStateCheck::Request>
  //       request, std::shared_ptr<wrp_ros::srv::LightStateCheck::Response>
  //       response);
};
}  // namespace westonrobot

#endif /* MOBILE_BASE_MESSENGER_HPP */
