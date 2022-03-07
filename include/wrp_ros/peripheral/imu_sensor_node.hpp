/*
 * imu_sensor_node.hpp
 *
 * Created on: Dec 20, 2021 08:45
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#ifndef IMU_SENSOR_NODE_HPP
#define IMU_SENSOR_NODE_HPP

#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "wrp_sdk/interface/imu_interface.hpp"

namespace westonrobot {
class ImuSensorNode {
 public:
  ImuSensorNode();
  ~ImuSensorNode();

 private:
  void PublishCallback(const ImuMsg& data);
  bool ReadParameters();

 private:
  std::string sensor_model_ = "wit";
  std::string device_path_ = "/dev/ttyUSB0";
  int baud_rate_ = 115200;
  std::string frame_id_ = "imu_link";

  ros::NodeHandle nh_;
  std::shared_ptr<ImuInterface> sensor_;
  ros::Publisher pub_;
  ros::Timer pub_timer_;
  sensor_msgs::Imu imu_data_;
};
}  // namespace westonrobot

#endif /* IMU_SENSOR_NODE_HPP */
