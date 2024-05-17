/*
 * power_regulator_node.hpp
 *
 * Created on: Apr 08, 2022 17:06
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#ifndef POWER_REGULATOR_NODE_HPP
#define POWER_REGULATOR_NODE_HPP

#include <memory>
#include <ros/ros.h>

#include "wrp_sdk_msgs/PowerRegulatorDeviceState.h"
#include "wrp_sdk_msgs/PowerRegulatorControl.h"

#include "wrp_sdk/interface/power_regulator_interface.hpp"

namespace westonrobot {
class PowerRegulatorNode {
 public:
  PowerRegulatorNode();
  ~PowerRegulatorNode();

 private:
  std::string device_path_ = "can0";

  ros::NodeHandle nh_;
  std::shared_ptr<PowerRegulatorInterface> regulator_;

  ros::Publisher state_pub_;
  ros::ServiceServer cmd_service_;

  void PublishCallback(const PowerRegulatorInterface::DeviceState& data);
  bool HandleCommand(wrp_sdk_msgs::PowerRegulatorControl::Request& req,
                     wrp_sdk_msgs::PowerRegulatorControl::Response& res);
  bool ReadParameters();
};
}  // namespace westonrobot

#endif /* POWER_REGULATOR_NODE_HPP */
