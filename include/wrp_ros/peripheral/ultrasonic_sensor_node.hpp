/*
 * ultrasonic_sensor_node.hpp
 *
 * Created on: Mar 04, 2022 18:03
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#ifndef ULTRASONIC_SENSOR_NODE_HPP
#define ULTRASONIC_SENSOR_NODE_HPP

#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>

#include "wrp_sdk/interface/ultrasonic_interface.hpp"

namespace westonrobot {
class UltrasonicSensorNode {
 public:
  UltrasonicSensorNode();
  ~UltrasonicSensorNode();

 private:
  std::string sensor_model_ = "dyp_a05";
  std::string device_path_ = "/dev/ttyUSB0";
  int baud_rate_ = 115200;
  std::string topic_name_ = "ultrasonic";
  std::string frame_id_ = "ultrasonic";

  ros::NodeHandle nh_;
  std::shared_ptr<UltrasonicInterface> sensor_;
  std::vector<ros::Publisher> pubs_;

  bool ReadParameters();
  void PublishCallback(const UltrasonicMsg& data);
};
}  // namespace westonrobot

#endif /* ULTRASONIC_SENSOR_NODE_HPP */
