/**
 * @file mobile_base_ros.cpp
 * @brief
 * @date 17-05-2024
 *
 * @copyright Copyright (c) 2024 Weston Robot Pte. Ltd.
 */
#include "wrp_sdk_robot/mobile_base_node.hpp"

#include <limits>

#include "wrp_sdk/mobile_base/westonrobot/mobile_base.hpp"
#include "wrp_sdk/mobile_base/agilex/agilex_base_v2_adapter.hpp"

#include <tf/transform_broadcaster.h>

#include "wrp_sdk_msgs/SystemState.h"
#include "wrp_sdk_msgs/MotionState.h"
#include "wrp_sdk_msgs/ActuatorStateArray.h"
#include "wrp_sdk_msgs/RangeData.h"

namespace westonrobot {
MobileBaseNode::MobileBaseNode(ros::NodeHandle* nh) : nh_(nh) {
  if (!MobileBaseNode::ReadParameters()) {
    ROS_ERROR("Could not load parameters");
    ros::shutdown();
  }

  if (!MobileBaseNode::SetupHardware()) {
    ROS_ERROR("Failed to setup robot");
    ros::shutdown();
  }

  if (!MobileBaseNode::SetupInterfaces()) {
    ROS_ERROR("Failed to setup ROS interfaces");
    ros::shutdown();
  }
}

bool MobileBaseNode::ReadParameters() {
  nh_->getParam("can_device", can_device_);
  nh_->getParam("robot_type", robot_type_);
  nh_->getParam("base_frame", base_frame_);
  nh_->getParam("odom_frame", odom_frame_);
  nh_->getParam("auto_reconnect", auto_reconnect_);
  nh_->getParam("motion_type", motion_type_);
  nh_->getParam("publish_odom_tf", publish_odom_tf_);

  ROS_INFO_STREAM("--- Parameters loaded are ---");
  ROS_INFO_STREAM("can_device: " << can_device_);
  ROS_INFO_STREAM("robot_type: " << robot_type_);
  ROS_INFO_STREAM("base_frame: " << base_frame_);
  ROS_INFO_STREAM("odom_frame: " << odom_frame_);
  ROS_INFO_STREAM("auto_reconnect: " << auto_reconnect_);
  ROS_INFO_STREAM("motion_type: " << motion_type_);
  ROS_INFO_STREAM("publish_odom_tf: " << publish_odom_tf_);
  ROS_INFO_STREAM("-----------------------------");

  return true;
}

bool MobileBaseNode::SetupHardware() {
  // create appropriate adapter
  if (robot_type_ == "weston") {
    robot_ = std::make_shared<MobileBase>();
  } else if (robot_type_ == "agilex") {
    robot_ = std::make_shared<AgilexBaseV2Adapter>();
  } else if (robot_type_ == "vbot") {
    robot_ = std::make_shared<MobileBase>(true);
  } else {
    ROS_ERROR(
        "Unknown robot base type\n Supported are \"weston\", "
        "\"agilex\" & \"vbot\"");
    return false;
  }

  // connect to robot through specified can device
  if (!robot_->Connect(can_device_)) {
    ROS_ERROR("Failed to connect to robot through port: %s",
              can_device_.c_str());
    return false;
  }

  return true;
}

bool MobileBaseNode::SetupInterfaces() {
  // publishers
  system_state_publisher_ =
      nh_->advertise<wrp_sdk_msgs::SystemState>("system_state", 10);
  motion_state_publisher_ =
      nh_->advertise<wrp_sdk_msgs::MotionState>("motion_state", 10);
  actuator_state_publisher_ =
      nh_->advertise<wrp_sdk_msgs::ActuatorStateArray>("actuator_state", 10);
  odom_publisher_ = nh_->advertise<nav_msgs::Odometry>("odom", 50);
  battery_state_publisher_ =
      nh_->advertise<sensor_msgs::BatteryState>("battery_state", 10);

  // subscribers
  motion_cmd_subscriber_ = nh_->subscribe<geometry_msgs::Twist>(
      "cmd_vel", 5, &MobileBaseNode::MotionCmdCallback, this);

  access_control_service_ = nh_->advertiseService(
      "access_control", &MobileBaseNode::HandleAccessControl, this);
  assisted_mode_control_service_ =
      nh_->advertiseService("assisted_mode_control",
                            &MobileBaseNode::HandleAssistedModeControl, this);
  light_control_service_ = nh_->advertiseService(
      "light_control", &MobileBaseNode::HandleLightControl, this);
  motion_reset_service_ = nh_->advertiseService(
      "motion_reset", &MobileBaseNode::HandleMotionReset, this);

  return true;
}

void MobileBaseNode::PublishRobotState() {
  auto system_state = robot_->GetSystemState();
  auto motion_state = robot_->GetMotionState();
  auto actuator_state = robot_->GetActuatorState();
  auto battery_state = robot_->GetBatteryState();

  // system state
  wrp_sdk_msgs::SystemState system_state_msg;
  system_state_msg.rc_connected = system_state.rc_connected;
  system_state_msg.error_code = static_cast<uint32_t>(system_state.error_code);
  system_state_msg.operational_state =
      static_cast<uint32_t>(system_state.operational_state);
  system_state_msg.control_state =
      static_cast<uint32_t>(system_state.control_state);

  system_state_publisher_.publish(system_state_msg);

  // motion state
  wrp_sdk_msgs::MotionState motion_state_msg;
  motion_state_msg.desired_linear.x = motion_state.desired_linear.x;
  motion_state_msg.desired_linear.y = motion_state.desired_linear.y;
  motion_state_msg.desired_linear.z = motion_state.desired_linear.z;
  motion_state_msg.desired_angular.x = motion_state.desired_angular.x;
  motion_state_msg.desired_angular.y = motion_state.desired_angular.y;
  motion_state_msg.desired_angular.z = motion_state.desired_angular.z;

  motion_state_msg.source = static_cast<uint32_t>(motion_state.source);
  motion_state_msg.collision_detected = motion_state.collision_detected;
  motion_state_msg.assisted_mode_enabled = motion_state.assisted_mode_enabled;

  motion_state_publisher_.publish(motion_state_msg);

  // actuator state
  wrp_sdk_msgs::ActuatorStateArray actuator_state_msg;
  for (int i = 0; i < actuator_state.size(); ++i) {
    wrp_sdk_msgs::ActuatorState actuator_msg;
    actuator_msg.id = actuator_state[i].id;
    actuator_msg.motor.rpm = actuator_state[i].motor.rpm;
    actuator_msg.motor.current = actuator_state[i].motor.current;
    actuator_msg.motor.pulse_count = actuator_state[i].motor.pulse_count;
    actuator_msg.driver.driver_voltage =
        actuator_state[i].driver.driver_voltage;
    actuator_msg.driver.driver_temperature =
        actuator_state[i].driver.driver_temperature;
    actuator_msg.driver.motor_temperature =
        actuator_state[i].driver.motor_temperature;
    actuator_msg.driver.driver_state = actuator_state[i].driver.driver_state;
    actuator_state_msg.states.push_back(actuator_msg);
  }
  actuator_state_publisher_.publish(actuator_state_msg);

  // battery state
  sensor_msgs::BatteryState battery_state_msg;
  battery_state_msg.header.stamp = ros::Time::now();
  battery_state_msg.voltage = battery_state.voltage;
  battery_state_msg.current = battery_state.current;
  battery_state_msg.charge = battery_state.charge;
  battery_state_msg.capacity = battery_state.capacity;
  battery_state_msg.design_capacity = battery_state.design_capacity;
  battery_state_msg.percentage = battery_state.percentage / 100.0f;
  battery_state_msg.power_supply_status =
      static_cast<uint8_t>(battery_state.power_supply_status);
  battery_state_msg.power_supply_health =
      static_cast<uint8_t>(battery_state.power_supply_health);
  battery_state_msg.power_supply_technology =
      static_cast<uint8_t>(battery_state.power_supply_technology);
  battery_state_msg.present = battery_state.present;
  battery_state_publisher_.publish(battery_state_msg);
}

void MobileBaseNode::PublishSensorData() {
  // TODO
}

void MobileBaseNode::PublishWheelOdometry() {
  auto robot_odom = robot_->GetOdometry();

  // ATTN: odometry directly from wrp_sdk still in progress
  geometry_msgs::Twist robot_twist;
  robot_twist.linear.x = robot_odom.linear.x;
  robot_twist.linear.y = robot_odom.linear.y;
  robot_twist.linear.z = robot_odom.linear.z;
  robot_twist.angular.x = robot_odom.angular.x;
  robot_twist.angular.y = robot_odom.angular.y;
  robot_twist.angular.z = robot_odom.angular.z;

  nav_msgs::Odometry odom_msg = MobileBaseNode::CalculateOdometry(robot_twist);

  // publish tf transformation
  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp = odom_msg.header.stamp;
  tf_msg.header.frame_id = odom_msg.header.frame_id;
  tf_msg.child_frame_id = odom_msg.child_frame_id;

  tf_msg.transform.translation.x = odom_msg.pose.pose.position.x;
  tf_msg.transform.translation.y = odom_msg.pose.pose.position.y;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation = odom_msg.pose.pose.orientation;

  if (publish_odom_tf_) {
    tf_broadcaster_.sendTransform(tf_msg);
  }
  odom_publisher_.publish(odom_msg);
}

nav_msgs::Odometry MobileBaseNode::CalculateOdometry(
    geometry_msgs::Twist robot_twist) {
  auto current_time = ros::Time::now();
  float dt = (current_time - last_time_).toSec();
  last_time_ = current_time;

  // TODO: perform calculation based on robot type & wheel base other than scout
  // & scout mini
  double linear_speed = robot_twist.linear.x;
  double angular_speed = robot_twist.angular.z;
  double lateral_speed = robot_twist.linear.y;

  double d_x =
      (linear_speed * std::cos(theta_) - lateral_speed * std::sin(theta_)) * dt;
  double d_y =
      (linear_speed * std::sin(theta_) + lateral_speed * std::cos(theta_)) * dt;
  double d_theta = angular_speed * dt;

  position_x_ += d_x;
  position_y_ += d_y;
  theta_ += d_theta;

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);

  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = current_time;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

  odom_msg.pose.pose.position.x = position_x_;
  odom_msg.pose.pose.position.y = position_y_;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;

  odom_msg.twist.twist.linear.x = linear_speed;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.angular.z = angular_speed;

  return odom_msg;
}

void MobileBaseNode::MotionCmdCallback(
    const geometry_msgs::Twist::ConstPtr& msg) {
  MotionCommand cmd;

  cmd.linear.x = msg->linear.x;
  cmd.linear.y = msg->linear.y;
  cmd.linear.z = msg->linear.z;
  cmd.angular.x = msg->angular.x;
  cmd.angular.y = msg->angular.y;
  cmd.angular.z = msg->angular.z;

  robot_->SetMotionCommand(cmd);
  //   ROS_INFO("CMD received:%f, %f", msg->linear.x, msg->angular.z);
}

bool MobileBaseNode::HandleAccessControl(
    wrp_sdk_msgs::AccessControl::Request& req,
    wrp_sdk_msgs::AccessControl::Response& res) {
  if (req.action_type ==
      wrp_sdk_msgs::AccessControl::Request::ACTION_TYPE_REQUEST_CONTROL) {
    auto result = robot_->RequestControl();
    res.result_code = static_cast<uint32_t>(result);
  } else if (req.action_type == wrp_sdk_msgs::AccessControl::Request::
                                    ACTION_TYPE_RENOUNCE_CONTROL) {
    auto result = robot_->RenounceControl();
    res.result_code = static_cast<uint32_t>(result);
  }
  return true;
}

bool MobileBaseNode::HandleAssistedModeControl(
    wrp_sdk_msgs::AssistedModeControl::Request& req,
    wrp_sdk_msgs::AssistedModeControl::Response& res) {
  AssistedModeSetCommand cmd;
  cmd.enable = req.enable;
  robot_->SetAssistedMode(cmd);
  res.state = req.enable;
  return true;
}

bool MobileBaseNode::HandleLightControl(
    wrp_sdk_msgs::LightControl::Request& req,
    wrp_sdk_msgs::LightControl::Response& res) {
  LightCommand cmd;

  cmd.id = req.id;
  cmd.command.mode = static_cast<LightMode>(req.command.mode);
  cmd.command.intensity = req.command.intensity;

  if (robot_->SdkHasControlToken()) {
    robot_->SetLightCommand(cmd);
  }
  auto light_state = robot_->GetLightState();
  res.state.mode = static_cast<uint32_t>(light_state.state.mode);
  res.state.intensity = light_state.state.intensity;
  return true;
}

bool MobileBaseNode::HandleMotionReset(
    wrp_sdk_msgs::MotionReset::Request& req,
    wrp_sdk_msgs::MotionReset::Response& res) {
  MotionResetCommand cmd;

  cmd.type = static_cast<MotionResetCommandType>(req.type);

  if (robot_->SdkHasControlToken()) {
    robot_->SetMotionResetCommand(cmd);
    res.result_code =
        wrp_sdk_msgs::MotionReset::Response::MOTION_RESET_SUCCCESS;
  } else {
    res.result_code = wrp_sdk_msgs::MotionReset::Response::MOTION_RESET_FAILURE;
  }

  return true;
}

void MobileBaseNode::Run(double loop_hz) {
  loop_period_ = 1.0 / loop_hz;
  ros::Rate rate(loop_hz);
  while (ros::ok()) {
    if (auto_reconnect_ && !robot_->SdkHasControlToken()) {
      auto ret = robot_->RequestControl();
      if (ret != HandshakeReturnCode::kControlAcquired) {
        std::cout << "Failed to gain control token, error code: "
                  << static_cast<int>(ret) << std::endl;
      }
    }

    PublishRobotState();
    PublishSensorData();
    PublishWheelOdometry();

    ros::spinOnce();
    rate.sleep();
  }
}
}  // namespace westonrobot

int main(int argc, char** argv) {
  // setup ROS node
  ros::init(argc, argv, "mobile_base_node");
  ros::NodeHandle node("~");

  westonrobot::MobileBaseNode base(&node);

  base.Run(50);

  return 0;
}