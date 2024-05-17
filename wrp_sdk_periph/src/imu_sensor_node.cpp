/*
 * imu_sensor_node.cpp
 *
 * Created on: Dec 20, 2021 08:47
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include "wrp_sdk_periph/peripheral/imu_sensor_node.hpp"

#include "wrp_sdk/peripheral/imu_sensor_wit.hpp"
#include "wrp_sdk/peripheral/imu_sensor_hipnuc.hpp"

namespace westonrobot {
ImuSensorNode::ImuSensorNode() {
  if (!ReadParameters()) {
    ROS_ERROR("Could not load parameters");
    ros::shutdown();
  }

  if (sensor_model_ == "hipnuc") {
    sensor_ = std::make_shared<ImuSensorHipnuc>();
  } else {
    sensor_ = std::make_shared<ImuSensorWit>();
  }

  if (!sensor_->Connect(device_path_, baud_rate_)) {
    ROS_ERROR("Failed to connect to IMU sensor: %s@%d", device_path_.c_str(),
              baud_rate_);
    ros::shutdown();
  }

  pub_ = nh_.advertise<sensor_msgs::Imu>("/imu", 1);

  // Callback publisher implementation
  sensor_->SetDataReceivedCallback(
      std::bind(&ImuSensorNode::PublishCallback, this, std::placeholders::_1));

  // Periodic timer publisher implementation
  // pub_timer_ =
  //     nh_.createTimer(ros::Duration(publish_interval_ / 1000.0),
  //                     std::bind(&ImuSensorNode::PublishCallback, this));
}

ImuSensorNode::~ImuSensorNode() { ros::shutdown(); }

void ImuSensorNode::PublishCallback(const ImuMsg& data) {
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
  }
  pub_.publish(imu_data_);
}

bool ImuSensorNode::ReadParameters() {
  nh_.getParam("sensor_model", sensor_model_);
  nh_.getParam("device_path", device_path_);
  nh_.getParam("baud_rate", baud_rate_);
  nh_.getParam("frame_id", frame_id_);

  if (sensor_model_ != "wit" && sensor_model_ != "hipnuc") {
    ROS_WARN("Invalid sensor model detected, defaulting to \"wit\"!!!!!!!!!");
    sensor_model_ = "wit";
  }

  ROS_INFO(
      "Successfully loaded the following parameters: \nDevice path: "
      "%s\nBaud rate: %d\nFrame id: %s",
      device_path_.c_str(), baud_rate_, frame_id_.c_str());
  return true;
}
}  // namespace westonrobot

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_sensor_node");
  ROS_INFO("imu sensor node running.");
  westonrobot::ImuSensorNode imu;
  ros::spin();
  return 0;
}