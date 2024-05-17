/*
 * ultrasonic_sensor_node.cpp
 *
 * Created on: Mar 04, 2022 18:57
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include "wrp_sdk_periph/ultrasonic_sensor_node.hpp"

#include "wrp_sdk/peripheral/ultrasonic_sensor_dyp.hpp"
#include "wrp_sdk/peripheral/ultrasonic_sensor_w200d.hpp"

namespace westonrobot {
UltrasonicSensorNode::UltrasonicSensorNode() {
  if (!ReadParameters()) {
    ROS_ERROR("Could not load parameters");
    ros::shutdown();
  }

  if (sensor_model_ == "w200d") {
    sensor_ = std::make_shared<UltrasonicSensorW200d>();
  } else {
    sensor_ = std::make_shared<UltrasonicSensorDyp>();
  }

  pubs_.resize(8);
  for (int i = 0; i < 8; ++i) {
    std::string topic_name = topic_name_ + std::to_string(i);
    pubs_[i] = nh_.advertise<sensor_msgs::Range>(topic_name, 10);
  }

  sensor_->SetDataReceivedCallback(std::bind(
      &UltrasonicSensorNode::PublishCallback, this, std::placeholders::_1));

  if (!sensor_->Connect(device_path_, baud_rate_)) {
    ROS_ERROR("Failed to connect to ultrasonic sensor: %s@%d",
              device_path_.c_str(), baud_rate_);
    ros::shutdown();
  }

}

UltrasonicSensorNode::~UltrasonicSensorNode() { ros::shutdown(); }

void UltrasonicSensorNode::PublishCallback(const UltrasonicMsg& msg) {
  sensor_msgs::Range range_msg;
  range_msg.header.stamp = ros::Time::now();

  for (int i = 0; i < msg.size(); ++i) {
    range_msg.header.frame_id = frame_id_ + std::to_string(i);
    range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg.field_of_view = msg[i].field_of_view;
    range_msg.min_range = msg[i].min_range;
    range_msg.max_range = msg[i].max_range;
    range_msg.range = msg[i].range;

    pubs_[i].publish(range_msg);
  }
}

bool UltrasonicSensorNode::ReadParameters() {
  nh_.getParam("sensor_model", sensor_model_);

  nh_.getParam("device_path", device_path_);
  nh_.getParam("baud_rate", baud_rate_);

  nh_.getParam("frame_id", frame_id_);
  nh_.getParam("topic_name", topic_name_);

  ROS_INFO(
      "Successfully loaded the following parameters: \nSensor model: %s\nDevice path: "
      "%s\nBaud rate: %d\nFrame id: %s",
      sensor_model_.c_str(), device_path_.c_str(), baud_rate_, frame_id_.c_str());
  return true;
}
}  // namespace westonrobot

int main(int argc, char** argv) {
  ros::init(argc, argv, "ultrasonic_sensor_node");
  ROS_INFO("ultrasonic sensor node running.");
  westonrobot::UltrasonicSensorNode ultrasonic;
  ros::spin();
  return 0;
}
