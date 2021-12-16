/*
 * mobile_base_messenger.cpp
 *
 * Created on: Jul 25, 2021 16:39
 * Description:
 *
 * Copyright (c) 2021 Weston Robot
 */

#include "wrp_ros/mobile_base_messenger.hpp"

#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/point_stamped.hpp>

namespace {
geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}
}  // namespace

namespace westonrobot {
MobileBaseROSMessenger::MobileBaseROSMessenger(
    std::string node_name, std::shared_ptr<MobileRobotInterface> robot)
    : robot_(robot),
      tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this)) {
  //   this->declare_parameter("can_port");
  //   this->declare_parameter("sim_control_rate");
  //   this->declare_parameter("is_simulated_robot");
  //   this->declare_parameter("odom_frame");
  //   this->declare_parameter("base_link_frame");
  //   this->declare_parameter("odom_topic");
  //   this->declare_parameter("linear_speed_tuning_factor");
  //   this->declare_parameter("angular_speed_tuning_factor");
  LoadParameters();
}

void MobileBaseROSMessenger::EnableSimMode() {
  simulated_robot_ = true;
  std::cout << "Enabled simulation mode" << std::endl;
}

void MobileBaseROSMessenger::LoadParameters() {
  //   this->get_parameter_or<std::string>("can_port", can_device_, "can0");
  //   this->get_parameter_or<int>("sim_control_rate", sim_control_rate_, 50);
  //   this->get_parameter_or<bool>("is_simulated_robot", simulated_robot_,
  //   false);

  //   this->get_parameter_or<std::string>("odom_frame", odom_frame_, "odom");
  //   this->get_parameter_or<std::string>("base_link_frame", base_frame_,
  //                                       "base_link");
  //   this->get_parameter_or<std::string>("odom_topic", odom_topic_name_,
  //   "odom");

  //   this->get_parameter_or<float>("linear_speed_tuning_factor",
  //                                 linear_speed_tuning_factor_, 1.0);
  //   this->get_parameter_or<float>("angular_speed_tuning_factor",
  //                                 angular_speed_tuning_factor_, 1.0);

  //   std::cout << "Loading parameters: " << std::endl;
  //   std::cout << "- can_port:" << can_device_ << std::endl;
  //   std::cout << "- sim_control_rate:" << sim_control_rate_ << std::endl;
  //   std::cout << "- is_simulated_robot:" << simulated_robot_ << std::endl;
  //   std::cout << "- odom_frame:" << odom_frame_ << std::endl;
  //   std::cout << "- base_link_frame:" << base_frame_ << std::endl;
  //   std::cout << "- odom_topic:" << odom_topic_name_ << std::endl;

  if (simulated_robot_) EnableSimMode();
}

void MobileBaseROSMessenger::SetOdomFrame(std::string odom_frame) {
  odom_frame_ = odom_frame;
}

void MobileBaseROSMessenger::SetBaseFrame(std::string base_frame) {
  base_frame_ = base_frame;
}

void MobileBaseROSMessenger::SetOdomTopicName(std::string odom_topic) {
  odom_topic_name_ = odom_topic;
}

void MobileBaseROSMessenger::SetupSubscription() {
  // publisher
  //   last_time_ = this->now();
  //   odom_publisher_ =
  //       this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name_,
  //       50);
  //   motion_state_publisher_ =
  //       this->create_publisher<robot_msgs::msg::MotionState>("/motion_state",
  //       10);
  //   ultrasonic_data_publisher_ =
  //       this->create_publisher<robot_msgs::msg::RangeData>("/ultrasonic_data",
  //                                                          10);
  //   tof_data_publisher_ =
  //       this->create_publisher<robot_msgs::msg::RangeData>("/tof_data", 10);

  //   // cmd subscriber
  //   motion_cmd_subscription_ =
  //       this->create_subscription<geometry_msgs::msg::Twist>(
  //           "/cmd_vel", 5,
  //           std::bind(&MobileBaseROSMessenger::TwistCmdCallback, this,
  //                     std::placeholders::_1));
  //   light_cmd_subscription_ =
  //       this->create_subscription<robot_msgs::msg::LightCommand>(
  //           "/light_control", 5,
  //           std::bind(&MobileBaseROSMessenger::LightCmdCallback, this,
  //                     std::placeholders::_1));
}

void MobileBaseROSMessenger::TwistCmdCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (!simulated_robot_) {
    MotionCommand motion_command;
    motion_command.linear = {0, 0, 0};
    motion_command.angular = {0, 0, 0};
    motion_command.linear.x = msg->linear.x;
    motion_command.angular.z = msg->angular.z;
    if (GetControlToken()) robot_->SetMotionCommand(motion_command);
  } else {
    std::lock_guard<std::mutex> guard(twist_mutex_);
    current_twist_ = *msg.get();
  }
  // ROS_INFO("cmd received:%f, %f", msg->linear.x, msg->angular.z);
}

void MobileBaseROSMessenger::LightCmdCallback(
    const robot_msgs::msg::LightCommand::SharedPtr msg) {
  if (!simulated_robot_) {
    LightCommand light_cmd;
    light_cmd.id = 0;
    light_cmd.command.mode = (LightMode)msg->command.mode;
    light_cmd.command.intensity = msg->command.intensity;
    robot_->SetLightCommand(light_cmd);
  } else {
    std::cout << "Simulated robot received light control cmd" << std::endl;
  }
}

void MobileBaseROSMessenger::PublishStateToROS() {
  current_time_ = this->now();

  double dt = (current_time_ - last_time_).seconds();

  static bool init_run = true;
  if (init_run) {
    last_time_ = current_time_;
    init_run = false;
    return;
  }

  // publish sstate message
  MotionState motion_state = robot_->GetMotionState();
  Odometry odometry = robot_->GetOdometry();
  RangeData ultrasonic_data = robot_->GetUltrasonicData();
  RangeData tof_data = robot_->GetTofData();

  robot_msgs::msg::MotionState motion_msg;
  robot_msgs::msg::RangeData ultrasonic_data_msg;
  robot_msgs::msg::RangeData tof_data_msg;

  motion_msg.desired_linear.x = motion_state.desired_linear.x;
  motion_msg.desired_linear.y = motion_state.desired_linear.y;
  motion_msg.desired_linear.z = motion_state.desired_linear.z;
  motion_msg.desired_angular.x = motion_state.desired_angular.x;
  motion_msg.desired_angular.y = motion_state.desired_angular.y;
  motion_msg.desired_angular.z = motion_state.desired_angular.z;

  motion_msg.source = (int)motion_state.source;
  motion_msg.collision_detected = motion_state.collision_detected;
  motion_msg.assisted_mode_enabled = motion_state.assisted_mode_enabled;

  ultrasonic_data_msg.type = (int)ultrasonic_data.type;

  for (int i = 0; i < 8; i++) {
    ultrasonic_data_msg.data[i].id = ultrasonic_data.data[i].id;
    ultrasonic_data_msg.data[i].threshold = ultrasonic_data.data[i].threshold;
    ultrasonic_data_msg.data[i].range = ultrasonic_data.data[i].range;
  }

  tof_data_msg.type = (int)tof_data.type;

  for (int i = 0; i < 2; i++) {
    tof_data_msg.data[i].id = tof_data.data[i].id;
    tof_data_msg.data[i].threshold = tof_data.data[i].threshold;
    tof_data_msg.data[i].range = tof_data.data[i].range;
  }

  motion_state_publisher_->publish(motion_msg);
  ultrasonic_data_publisher_->publish(ultrasonic_data_msg);
  tof_data_publisher_->publish(tof_data_msg);

  auto system_state = robot_->GetSystemState();
  // publish odometry and tf
  if (system_state.operational_state != SystemOperationalState::kEStopActivated)
    PublishOdometryToROS(odometry.linear.x, odometry.angular.z, dt);

  // record time for next integration
  last_time_ = current_time_;
}

void MobileBaseROSMessenger::PublishSimStateToROS(double linear,
                                                  double angular) {
  current_time_ = this->now();
  double dt = (current_time_ - last_time_).seconds();

  static bool init_run = true;
  if (init_run) {
    last_time_ = current_time_;
    init_run = false;
    return;
  }

  // publish odometry and tf
  PublishOdometryToROS(linear, angular, dt);

  // record time for next integration
  last_time_ = current_time_;
}

void MobileBaseROSMessenger::PublishOdometryToROS(double linear, double angular,
                                                  double dt) {
  // perform numerical integration to get an estimation of pose
  // RCLCPP_INFO(this->get_logger(), "PUBLISHING ODOM, dt: " +
  // std::to_string(dt)
  // +
  //                                    "linear: " + std::to_string(linear) +
  //                                    "angular: " + std::to_string(angular));
  linear_speed_ = linear * linear_speed_tuning_factor_;
  angular_speed_ = angular * angular_speed_tuning_factor_;

  double d_x = linear_speed_ * std::cos(theta_) * dt;
  double d_y = linear_speed_ * std::sin(theta_) * dt;
  double d_theta = angular_speed_ * dt;

  position_x_ += d_x;
  position_y_ += d_y;
  theta_ += d_theta;

  geometry_msgs::msg::Quaternion odom_quat = createQuaternionMsgFromYaw(theta_);

  // publish tf transformation
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = current_time_;
  tf_msg.header.frame_id = odom_frame_;
  tf_msg.child_frame_id = base_frame_;

  tf_msg.transform.translation.x = position_x_;
  tf_msg.transform.translation.y = position_y_;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation = odom_quat;

  tf_broadcaster_->sendTransform(tf_msg);

  // publish odometry and tf messages
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = current_time_;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

  odom_msg.pose.pose.position.x = position_x_;
  odom_msg.pose.pose.position.y = position_y_;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;

  odom_msg.twist.twist.linear.x = linear_speed_;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.angular.z = angular_speed_;

  odom_publisher_->publish(odom_msg);
}

bool MobileBaseROSMessenger::GetControlToken() {
  if (robot_->SdkHasControlToken()) return true;
  HandshakeReturnCode feedback;
  feedback = robot_->RequestControl();
  if (feedback == HandshakeReturnCode::kControlAcquired) {
    return true;
  } else if (feedback == HandshakeReturnCode::kAlreadyGainedControl) {
    return true;
  }
  return false;
}

void MobileBaseROSMessenger::SetupServices() {
  request_control_service_ =
      this->create_service<robot_msgs::srv::RequestControl>(
          "/request_robot_control",
          std::bind(&MobileBaseROSMessenger::RequestControlCallback, this,
                    std::placeholders::_1, std::placeholders::_2));
  renounce_control_service_ =
      this->create_service<robot_msgs::srv::RenounceControl>(
          "/renounce_robot_control",
          std::bind(&MobileBaseROSMessenger::RenounceControlCallback, this,
                    std::placeholders::_1, std::placeholders::_2));
  system_check_service_ =
      this->create_service<robot_msgs::srv::SystemStateCheck>(
          "/system_state_check",
          std::bind(&MobileBaseROSMessenger::SystemStateCallback, this,
                    std::placeholders::_1, std::placeholders::_2));
  rc_check_service_ = this->create_service<robot_msgs::srv::RcStateCheck>(
      "/rc_state_check",
      std::bind(&MobileBaseROSMessenger::RcStateCheckCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  actuator_check_service_ =
      this->create_service<robot_msgs::srv::ActuatorStateCheck>(
          "/actuator_state_check",
          std::bind(&MobileBaseROSMessenger::ActuatorStateCheckCallback, this,
                    std::placeholders::_1, std::placeholders::_2));
  light_check_service_ = this->create_service<robot_msgs::srv::LightStateCheck>(
      "/light_state_check",
      std::bind(&MobileBaseROSMessenger::LightStateCheckCallback, this,
                std::placeholders::_1, std::placeholders::_2));
}

void MobileBaseROSMessenger::RequestControlCallback(
    const std::shared_ptr<robot_msgs::srv::RequestControl::Request> request,
    std::shared_ptr<robot_msgs::srv::RequestControl::Response> response) {
  (void)request;
  auto feedback = robot_->RequestControl();
  switch (feedback) {
    case HandshakeReturnCode::kRobotBaseNotAlive:
      std::cout << "RobotBaseNotAlive" << std::endl;
      break;
    case HandshakeReturnCode::kAlreadyGainedControl:
      std::cout << "AlreadyGainedControl" << std::endl;
      break;
    case HandshakeReturnCode::kAlreadyLostControl:
      std::cout << "AlreadyLostControl" << std::endl;
      break;
    case HandshakeReturnCode::kControlAcquired:
      std::cout << "ControlAcquired" << std::endl;
      break;
    case HandshakeReturnCode::kControlRejected_RobotBaseFault:
      std::cout << "ControlRejected_RobotBaseFault" << std::endl;
      break;
    case HandshakeReturnCode::kControlRejected_RcHaltTriggered:
      std::cout << "ControlRejected_RcHaltTriggered" << std::endl;
      break;
    case HandshakeReturnCode::kControlRejected_RcControlActive:
      std::cout << "ControlRejected_RcControlActive" << std::endl;
      break;
    case HandshakeReturnCode::kControlRejected_TokenTransferInterrupted:
      std::cout << "ControlRejected_TokenTransferInterrupted" << std::endl;
      break;
    case HandshakeReturnCode::kControlRequestTimeout:
      std::cout << "ControlRequestTimeout" << std::endl;
      break;
    default:
      std::cout << "Invalid feedback from request control" << std::endl;
      break;
  }
  response->result_code = (int)feedback;
}

void MobileBaseROSMessenger::RenounceControlCallback(
    const std::shared_ptr<robot_msgs::srv::RenounceControl::Request> request,
    std::shared_ptr<robot_msgs::srv::RenounceControl::Response> response) {
  (void)request;
  auto feedback = robot_->RenounceControl();
  switch (feedback) {
    case HandshakeReturnCode::kControlHandedOver:
      std::cout << "ControlHandedOver" << std::endl;
      break;
    case HandshakeReturnCode::kRenounceRequestTimeout:
      std::cout << "RenounceRequestTimeout" << std::endl;
      break;
    default:
      std::cout << "Invalid feedback from renounce control" << std::endl;
      break;
  }
  response->result_code = (int)feedback;
}

void MobileBaseROSMessenger::SystemStateCallback(
    const std::shared_ptr<robot_msgs::srv::SystemStateCheck::Request> request,
    std::shared_ptr<robot_msgs::srv::SystemStateCheck::Response> response) {
  (void)request;
  auto system_state = robot_->GetSystemState();
  response->rc_connected = system_state.rc_connected;
  response->error_code = (uint32_t)system_state.error_code;
  response->operational_state = (int)system_state.operational_state;
  response->control_state = (int)system_state.control_state;
}

void MobileBaseROSMessenger::RcStateCheckCallback(
    const std::shared_ptr<robot_msgs::srv::RcStateCheck::Request> request,
    std::shared_ptr<robot_msgs::srv::RcStateCheck::Response> response) {
  (void)request;
  auto rc_state = robot_->GetRcState();

  for (int i = 0; i < 8; i++) {
    response->axes[i] = rc_state.axes[i];
    response->buttons[i] = rc_state.buttons[i];
  }
}

void MobileBaseROSMessenger::ActuatorStateCheckCallback(
    const std::shared_ptr<robot_msgs::srv::ActuatorStateCheck::Request> request,
    std::shared_ptr<robot_msgs::srv::ActuatorStateCheck::Response> response) {
  auto actuator_state = robot_->GetActuatorState();
  uint8_t id = request->id;

  for (int i = 0; i < 4; i++) {
    if (actuator_state[i].id == id) {
      response->actuator_id = id;
      response->rpm = actuator_state[i].motor.rpm;
      response->current = actuator_state[i].motor.current;
      response->pulse_count = actuator_state[i].motor.pulse_count;
      response->driver_voltage = actuator_state[i].driver.driver_voltage;
      response->driver_temperature =
          actuator_state[i].driver.driver_temperature;
      response->motor_temperature = actuator_state[i].driver.motor_temperature;
      response->driver_state = actuator_state[i].driver.driver_state;
    }
  }
}

void MobileBaseROSMessenger::LightStateCheckCallback(
    const std::shared_ptr<robot_msgs::srv::LightStateCheck::Request> request,
    std::shared_ptr<robot_msgs::srv::LightStateCheck::Response> response) {
  LightCommand light_command;
  light_command.id = request->id;
  light_command.command.mode = LightMode::kEnquiry;
  robot_->SetLightCommand(light_command);

  auto light_state = robot_->GetLightState();

  response->mode = (int)light_state.state.mode;
  response->intensity = light_state.state.intensity;
}
}  // namespace westonrobot