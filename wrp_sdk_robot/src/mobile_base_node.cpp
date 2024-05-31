/**
 * @file mobile_base_ros.cpp
 * @brief
 * @date 17-05-2024
 *
 * @copyright Copyright (c) 2024 Weston Robot Pte. Ltd.
 */
#include <limits>

#include "wrp_sdk_robot/mobile_base_node.hpp"
#include "wrp_sdk_robot/kinematic_models.hpp"

#include "wrp_sdk/mobile_base/westonrobot/mobile_base.hpp"
#include "wrp_sdk/mobile_base/agilex/agilex_base_v2_adapter.hpp"
#include "wrp_sdk/mobile_base/bangbang/robooterx_base_adapter.hpp"

#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
  nh_->getParam("robot_type", robot_type_);
  nh_->getParam("can_device", can_device_);
  nh_->getParam("base_frame", base_frame_);
  nh_->getParam("odom_frame", odom_frame_);
  nh_->getParam("publish_odom_tf", publish_odom_tf_);
  nh_->getParam("auto_reconnect", auto_reconnect_);

  if (robot_type_ < 0 ||
      robot_type_ >= static_cast<int>(RobotVariant::kNumOfVariants)) {
    ROS_ERROR("Invalid robot type");
    return false;
  }
  robot_variant_ = static_cast<RobotVariant>(robot_type_);

  ROS_INFO_STREAM("--- Parameters loaded are ---");
  ROS_INFO_STREAM("robot_type: " << robot_type_);
  ROS_INFO_STREAM("can_device: " << can_device_);
  ROS_INFO_STREAM("base_frame: " << base_frame_);
  ROS_INFO_STREAM("odom_frame: " << odom_frame_);
  ROS_INFO_STREAM("publish_odom_tf: " << publish_odom_tf_);
  ROS_INFO_STREAM("auto_reconnect: " << auto_reconnect_);
  ROS_INFO_STREAM("-----------------------------");

  return true;
}

bool MobileBaseNode::SetupHardware() {
  // Create appropriate adaptor
  switch (robot_variant_) {
    case RobotVariant::kWRScout: {
      robot_ = std::make_shared<MobileBase>();
      break;
    }
    case RobotVariant::kWRVBot: {
      robot_ = std::make_shared<MobileBase>(true);
      break;
    }
    case RobotVariant::kAgilexScoutV2:
    case RobotVariant::kAgilexScoutMini:
    case RobotVariant::kAgilexScoutMiniOmni:
    case RobotVariant::kAgilexRangerMiniV1:
    case RobotVariant::kAgilexRangerMiniV2:
    case RobotVariant::kAgilexTracer:
    case RobotVariant::kAgilexTracerMini:
    case RobotVariant::kAgilexHunter:
    case RobotVariant::kAgilexHunterSE:
    case RobotVariant::kAgilexBunker: {
      robot_ = std::make_shared<AgilexBaseV2Adapter>();
      break;
    }
    case RobotVariant::kBangBangRobooterX: {
      robot_ = std::make_shared<RobooterXBaseAdapter>();
      break;
    }
    default: {
      ROS_ERROR_STREAM("Unknown robot base type");
      return false;
    }
  }

  // Connect to robot through can device
  if (!robot_->Connect(can_device_)) {
    ROS_ERROR_STREAM(
        "Failed to connect to robot through port: " << can_device_);
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
  battery_state_publisher_ =
      nh_->advertise<sensor_msgs::BatteryState>("battery_state", 10);
  rc_state_publisher_ = nh_->advertise<wrp_sdk_msgs::RcState>("rc_state", 10);
  odom_publisher_ = nh_->advertise<nav_msgs::Odometry>("odom", 50);

  // subscribers
  motion_cmd_subscriber_ = nh_->subscribe<geometry_msgs::Twist>(
      "cmd_vel", 5, &MobileBaseNode::MotionCmdCallback, this);

  access_control_service_ = nh_->advertiseService(
      "access_control", &MobileBaseNode::AccessControlCallback, this);
  assisted_mode_control_service_ =
      nh_->advertiseService("assisted_mode_control",
                            &MobileBaseNode::AssistedModeControlCallback, this);
  light_control_service_ = nh_->advertiseService(
      "light_control", &MobileBaseNode::LightControlCallback, this);
  motion_reset_service_ = nh_->advertiseService(
      "motion_reset", &MobileBaseNode::MotionResetCallback, this);

  std::lock_guard<std::mutex> lock(odom_mutex_);
  last_odom_time_ = ros::Time::now();

  return true;
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

  if (robot_->SdkHasControlToken()) {
    robot_->SetMotionCommand(cmd);
  }
}

bool MobileBaseNode::AccessControlCallback(
    wrp_sdk_msgs::AccessControl::Request& req,
    wrp_sdk_msgs::AccessControl::Response& res) {
  HandshakeReturnCode result;
  switch (req.action_type) {
    case wrp_sdk_msgs::AccessControl::Request::ACTION_TYPE_REQUEST_CONTROL: {
      result = robot_->RequestControl();
      res.result_code = static_cast<uint32_t>(result);
      break;
    }
    case wrp_sdk_msgs::AccessControl::Request::ACTION_TYPE_RENOUNCE_CONTROL: {
      result = robot_->RenounceControl();
      res.result_code = static_cast<uint32_t>(result);
      break;
    }

    default: {
      res.result_code = 11;
      break;
    }
  }
  return true;
}

bool MobileBaseNode::AssistedModeControlCallback(
    wrp_sdk_msgs::AssistedModeControl::Request& req,
    wrp_sdk_msgs::AssistedModeControl::Response& res) {
  AssistedModeSetCommand cmd;
  cmd.enable = req.enable;
  robot_->SetAssistedMode(cmd);
  res.state = req.enable;
  return true;
}

bool MobileBaseNode::LightControlCallback(
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

bool MobileBaseNode::MotionResetCallback(
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

void MobileBaseNode::PublishSystemState() {
  auto system_state = robot_->GetSystemState();
  auto motion_state = robot_->GetMotionState();

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
}

void MobileBaseNode::PublishBatteryState() {
  auto battery_state = robot_->GetBatteryState();

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

void MobileBaseNode::PublishActuatorState() {
  auto actuator_state = robot_->GetActuatorState();

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
}

void MobileBaseNode::PublishRcState() {
  auto rc_state = robot_->GetRcState();

  wrp_sdk_msgs::RcState rc_state_msg;
  rc_state_msg.header.stamp = ros::Time::now();
  rc_state_msg.header.frame_id = base_frame_;

  for (size_t i = 0; i < 8; i++) {
    rc_state_msg.axes[i] = rc_state.axes[i];
  }

  for (size_t i = 0; i < 8; i++) {
    rc_state_msg.buttons[i] = rc_state.buttons[i];
  }

  rc_state_publisher_.publish(rc_state_msg);
}

void MobileBaseNode::PublishOdometry() {
  auto robot_odom = robot_->GetOdometry();

  geometry_msgs::Twist robot_twist;
  robot_twist.linear.x = robot_odom.linear.x;
  robot_twist.linear.y = robot_odom.linear.y;
  robot_twist.linear.z = robot_odom.linear.z;
  robot_twist.angular.x = robot_odom.angular.x;
  robot_twist.angular.y = robot_odom.angular.y;
  robot_twist.angular.z = robot_odom.angular.z;

  MobileBaseNode::UpdateOdometry(robot_twist);

  std::lock_guard<std::mutex> lock(odom_mutex_);

  // publish tf transformation
  if (publish_odom_tf_) {
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = odom_msg_.header.stamp;
    tf_msg.header.frame_id = odom_msg_.header.frame_id;
    tf_msg.child_frame_id = odom_msg_.child_frame_id;

    tf_msg.transform.translation.x = odom_msg_.pose.pose.position.x;
    tf_msg.transform.translation.y = odom_msg_.pose.pose.position.y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_msg_.pose.pose.orientation;

    tf_broadcaster_.sendTransform(tf_msg);
  }

  odom_publisher_.publish(odom_msg_);
}

void MobileBaseNode::UpdateOdometry(geometry_msgs::Twist twist) {
  nav_msgs::Odometry odom_msg;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_msg = odom_msg_;
  }
  // Set twist
  odom_msg.twist.twist = twist;

  // Get dt
  ros::Time current_time = ros::Time::now();
  double dt = (current_time - last_odom_time_).toSec();
  last_odom_time_ = current_time;

  // Get current pose as RPY
  tf2::Quaternion current_quat;
  tf2::fromMsg(odom_msg.pose.pose.orientation, current_quat);
  tf2::Matrix3x3 m(current_quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Update pose
  switch (robot_variant_) {
    case RobotVariant::kAgilexScoutV2:
    case RobotVariant::kAgilexScoutMini:
    case RobotVariant::kAgilexTracer:
    case RobotVariant::kAgilexTracerMini:
    case RobotVariant::kAgilexBunker:
    case RobotVariant::kWRScout:
    case RobotVariant::kWRVBot:
    case RobotVariant::kBangBangRobooterX: {  // Differential drive platforms
      MotionModel<DifferentialModel> model;
      DifferentialModel::StateType current_state = {
          odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, yaw};
      DifferentialModel::ControlType control = {twist.linear.x,
                                                twist.angular.z};
      DifferentialModel::ParamType param = {};
      auto new_state = model.StepForward(current_state, control, param, dt);
      odom_msg.pose.pose.position.x = new_state[0];
      odom_msg.pose.pose.position.y = new_state[1];
      current_quat.setRPY(0.0, 0.0, new_state[2]);
      odom_msg.pose.pose.orientation = tf2::toMsg(current_quat);
      break;
    }
    case RobotVariant::kAgilexScoutMiniOmni: {  // Omni drive platforms
      MotionModel<OmniModel> model;
      OmniModel::StateType current_state = {odom_msg.pose.pose.position.x,
                                            odom_msg.pose.pose.position.y, yaw};
      OmniModel::ControlType control = {twist.linear.x, twist.linear.y,
                                        twist.angular.z};
      OmniModel::ParamType param = {};
      auto new_state = model.StepForward(current_state, control, param, dt);
      odom_msg.pose.pose.position.x = new_state[0];
      odom_msg.pose.pose.position.y = new_state[1];
      current_quat.setRPY(0.0, 0.0, new_state[2]);
      odom_msg.pose.pose.orientation = tf2::toMsg(current_quat);
      break;
    }
    case RobotVariant::kAgilexHunter:
    case RobotVariant::kAgilexHunterSE: {  // Ackermann drive platforms
      // TODO: Implement Ackermann drive model
      break;
    }
    case RobotVariant::kAgilexRanger:
    case RobotVariant::kAgilexRangerMiniV1:
    case RobotVariant::kAgilexRangerMiniV2: {  // Multi-modal drive platforms
      // TODO: Implement Multi-modal drive model
      break;
    }
    default: {
      ROS_ERROR_STREAM("Unknown robot type: " << robot_type_);
      return;
    }
  }
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_msg_ = odom_msg;
  }
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

    PublishSystemState();
    PublishBatteryState();
    PublishActuatorState();
    PublishOdometry();
    PublishRcState();

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