/*
 * imu_sensor_node.cpp
 *
 * Created on: Dec 20, 2021 08:47
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include "wrp_ros/peripheral/imu_sensor_node.hpp"

namespace westonrobot {
namespace wrp_ros {
ImuSensorNode::ImuSensorNode() {
  if (!ReadParameters()) {
    ROS_ERROR("Could not load parameters");
    ros::shutdown();
  }

  sensor_ = std::make_shared<ImuSensor>();

  if (!sensor_->Connect(device_path_, baud_rate_)) {
    ROS_ERROR("Failed to connect to IMU sensor: %s@%d", device_path_.c_str(),
              baud_rate_);
    ros::shutdown();
  }

  pub_ = nh_.advertise<sensor_msgs::Imu>("imu_sensor/imu", 1);

  // Callback publisher implementation
  sensor_->SetDataReceivedCallback(
      std::bind(&ImuSensorNode::PublishCallback, this, std::placeholders::_1));

  // Periodic timer publisher implementation
  // pub_timer_ =
  //     nh_.createTimer(ros::Duration(publish_interval_ / 1000.0),
  //                     std::bind(&ImuSensorNode::PublishCallback, this));
}

ImuSensorNode::~ImuSensorNode() { ros::shutdown(); }

void ImuSensorNode::PublishCallback(const ImuData& data) {
  imu_data_.header.stamp = ros::Time::now();
  imu_data_.header.frame_id = frame_id_;
  imu_data_.orientation.x = data.orientation.x;
  imu_data_.orientation.y = data.orientation.y;
  imu_data_.orientation.z = data.orientation.z;
  imu_data_.orientation.w = data.orientation.w;
  for (int i = 0; i < 9; ++i) {
    imu_data_.orientation_covariance[i] = data.orientation_covariance[i];
  }
  imu_data_.angular_velocity.x = data.angular_velocity.x;
  imu_data_.angular_velocity.y = data.angular_velocity.y;
  imu_data_.angular_velocity.z = data.angular_velocity.z;
  for (int i = 0; i < 9; ++i) {
    imu_data_.angular_velocity_covariance[i] =
        data.angular_velocity_covariance[i];
  }
  imu_data_.linear_acceleration.x = data.linear_acceleration.x;
  imu_data_.linear_acceleration.y = data.linear_acceleration.y;
  imu_data_.linear_acceleration.z = data.linear_acceleration.z;
  for (int i = 0; i < 9; ++i) {
    imu_data_.linear_acceleration_covariance[i] =
        data.linear_acceleration_covariance[i];

    pub_.publish(imu_data_);
  }
}

bool ImuSensorNode::ReadParameters() {
  nh_.getParam("device_path", device_path_);
  nh_.getParam("publish_interval", publish_interval_);
  nh_.getParam("baud_rate", baud_rate_);
  nh_.getParam("frame_id", frame_id_);
  ROS_INFO(
      "Successfully loaded the following parameters: \nDevice path: "
      "%s\nPublish interval: "
      "%d\nBaud rate: %d\nFrame id: %s",
      device_path_.c_str(), publish_interval_, baud_rate_, frame_id_.c_str());
  return true;
}
}  // namespace wrp_ros
}  // namespace westonrobot

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_sensor_node");
  ROS_INFO("imu sensor node running.");
  westonrobot::wrp_ros::ImuSensorNode imu;
  ros::spin();
  return 0;
}