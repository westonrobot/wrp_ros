/*
 * gps_receiver_node.cpp
 *
 * Created on: Dec 20, 2021 08:47
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include "wrp_ros/peripheral/gps_receiver_node.hpp"

#include "wrp_sdk/peripheral/gps_receiver_nmea.hpp"

namespace westonrobot {
GpsReceiverNode::GpsReceiverNode() {
  if (!ReadParameters()) {
    ROS_ERROR("Could not load parameters");
    ros::shutdown();
  }

  receiver_ = std::make_shared<GpsReceiverNmea>();

  if (!receiver_->Connect(device_path_, baud_rate_)) {
    ROS_ERROR("Failed to connect to GPS port: %s@%d", device_path_.c_str(),
              baud_rate_);
    ros::shutdown();
  }

  pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/fix", 1);

  // Callback publisher implementation
  receiver_->SetDataReceivedCallback(std::bind(
      &GpsReceiverNode::PublishCallback, this, std::placeholders::_1));

  // Periodic timer publisher implementation
  // pub_timer_ =
  //     nh_.createTimer(ros::Duration(publish_interval_ / 1000.0),
  //                     std::bind(&GpsReceiverNode::PublishCallback, this));
}

GpsReceiverNode::~GpsReceiverNode() { ros::shutdown(); }

bool GpsReceiverNode::ReadParameters() {
  nh_.getParam("device_path", device_path_);
  nh_.getParam("baud_rate", baud_rate_);

  nh_.getParam("frame_id", frame_id_);

  ROS_INFO(
      "Successfully loaded the following parameters: \nDevice path: "
      "%s\nBaud rate: %d\nFrame id: %s",
      device_path_.c_str(), baud_rate_, frame_id_.c_str());
  return true;
}

void GpsReceiverNode::PublishCallback(const NavSatFixMsg& gps_fix) {
  sat_fix_.header.stamp = ros::Time::now();
  sat_fix_.header.frame_id = frame_id_;
  sat_fix_.status.status = gps_fix.status.status;
  sat_fix_.status.service = gps_fix.status.service;
  sat_fix_.latitude = gps_fix.latitude;
  sat_fix_.longitude = gps_fix.longitude;
  sat_fix_.altitude = gps_fix.altitude;
  for (int i = 0; i < 9; ++i) {
    sat_fix_.position_covariance[i] = gps_fix.position_covariance[i];
  }
  pub_.publish(sat_fix_);
}
}  // namespace westonrobot

int main(int argc, char** argv) {
  ros::init(argc, argv, "gps_receiver_node");
  ROS_INFO("gps receiver node running.");
  westonrobot::GpsReceiverNode gps;
  ros::spin();
  return 0;
}