/*
 * power_regulator_node.cpp
 *
 * Created on: Apr 11, 2022 10:01
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include "wrp_sdk_periph/peripheral/power_regulator_node.hpp"

#include "wrp_sdk/peripheral/power_regulator_v2.hpp"

namespace westonrobot {
PowerRegulatorNode::PowerRegulatorNode() {
  if (!ReadParameters()) {
    ROS_ERROR("Could not load parameters");
    ros::shutdown();
  }

  regulator_ = std::unique_ptr<PowerRegulatorV2>(new PowerRegulatorV2());

  if (!regulator_->Connect(device_path_)) {
    ROS_ERROR("Failed to connect to power regulator: %s", device_path_.c_str());
    ros::shutdown();
  }

  state_pub_ = nh_.advertise<wrp_sdk_periph::PowerRegulatorDeviceState>(
      "/power_regulator/state", 5);

  cmd_service_ = nh_.advertiseService("/power_regulator/cmd",
                                      &PowerRegulatorNode::HandleCommand, this);

  regulator_->SetDataReceivedCallback(std::bind(
      &PowerRegulatorNode::PublishCallback, this, std::placeholders::_1));
}

PowerRegulatorNode::~PowerRegulatorNode() { ros::shutdown(); }

bool PowerRegulatorNode::HandleCommand(
    wrp_sdk_periph::PowerRegulatorControl::Request& req,
    wrp_sdk_periph::PowerRegulatorControl::Response& res) {
  ROS_INFO("Power regulator channel: %d, %d", req.channel, req.enable);

  PowerRegulatorInterface::OutputChannel chn;
  if (req.channel == wrp_sdk_periph::PowerRegulatorControl::Request::CHANNEL_19V) {
    chn = PowerRegulatorV2::kChannel19V;
  } else if (req.channel ==
             wrp_sdk_periph::PowerRegulatorControl::Request::CHANNEL_12V) {
    chn = PowerRegulatorV2::kChannel12V;
  } else if (req.channel ==
             wrp_sdk_periph::PowerRegulatorControl::Request::CHANNEL_5VI) {
    chn = PowerRegulatorV2::kChannel5Vi;
  } else if (req.channel ==
             wrp_sdk_periph::PowerRegulatorControl::Request::CHANNEL_12VI) {
    chn = PowerRegulatorV2::kChannel12Vi;
  } else {
    res.state = false;
  }

  regulator_->SetChannelState(chn, req.enable);
  res.state = true;

  return res.state;
}

void PowerRegulatorNode::PublishCallback(
    const PowerRegulatorInterface::DeviceState& data) {
  //   ROS_INFO("regulator msg received");

  wrp_sdk_periph::PowerRegulatorDeviceState state_msg;

  state_msg.input_voltage = data.input_voltage;
  state_msg.fan_speed = data.fan_speed;
  state_msg.temperature = data.temperature;

  state_msg.channels[0].name = "19V";
  state_msg.channels[0].enabled =
      data.channels.at(PowerRegulatorV2::kChannel19V).enabled;
  state_msg.channels[0].voltage =
      data.channels.at(PowerRegulatorV2::kChannel19V).voltage;
  state_msg.channels[0].current =
      data.channels.at(PowerRegulatorV2::kChannel19V).current;

  state_msg.channels[1].name = "12V";
  state_msg.channels[1].enabled =
      data.channels.at(PowerRegulatorV2::kChannel12V).enabled;
  state_msg.channels[1].voltage =
      data.channels.at(PowerRegulatorV2::kChannel12V).voltage;
  state_msg.channels[1].current =
      data.channels.at(PowerRegulatorV2::kChannel12V).current;

  state_msg.channels[2].name = "5V isolated";
  state_msg.channels[2].enabled =
      data.channels.at(PowerRegulatorV2::kChannel5Vi).enabled;
  state_msg.channels[2].voltage =
      data.channels.at(PowerRegulatorV2::kChannel5Vi).voltage;
  state_msg.channels[2].current =
      data.channels.at(PowerRegulatorV2::kChannel5Vi).current;

  state_msg.channels[3].name = "12V isolated";
  state_msg.channels[3].enabled =
      data.channels.at(PowerRegulatorV2::kChannel12Vi).enabled;
  state_msg.channels[3].voltage =
      data.channels.at(PowerRegulatorV2::kChannel12Vi).voltage;
  state_msg.channels[3].current =
      data.channels.at(PowerRegulatorV2::kChannel12Vi).current;

  state_pub_.publish(state_msg);
}

bool PowerRegulatorNode::ReadParameters() {
  nh_.getParam("device_path", device_path_);

  ROS_INFO("Successfully loaded the following parameters: \nDevice path: %s\n",
           device_path_.c_str());

  return true;
}
}  // namespace westonrobot

int main(int argc, char** argv) {
  ros::init(argc, argv, "power_regulator_node");
  ROS_INFO("power regulator node running.");

  westonrobot::PowerRegulatorNode regulator;
  ros::spin();
  return 0;
}
