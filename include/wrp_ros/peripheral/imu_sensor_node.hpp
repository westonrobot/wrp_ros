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

#include "wrp_sdk/peripheral/imu_sensor.hpp"

namespace westonrobot {
namespace wrp_ros {
class ImuSensorNode {
 public:
  ImuSensorNode();
  ~ImuSensorNode();

 private:
  void PublishCallback(const ImuData& data);
  bool ReadParameters();

 private:
  ros::NodeHandle nh_;
  std::shared_ptr<ImuSensor> sensor_;
  ros::Publisher pub_;
  ros::Timer pub_timer_;
  sensor_msgs::Imu imu_data_;
  std::string device_path_ = "/dev/ttyUSB0";
  int publish_interval_ = 500;
  int baud_rate_ = 115200;

  std::string topic_name_ = "imu";
  std::string frame_id_ = "imu_link";
};
}  // namespace wrp_ros
}  // namespace westonrobot

#endif /* IMU_SENSOR_NODE_HPP */
