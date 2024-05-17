/*
 * gps_receiver_node.hpp
 *
 * Created on: Dec 20, 2021 08:45
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#ifndef GPS_RECEIVER_NODE_HPP
#define GPS_RECEIVER_NODE_HPP

#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include "wrp_sdk/interface/gps_interface.hpp"

namespace westonrobot {
class GpsReceiverNode {
 public:
  GpsReceiverNode();
  ~GpsReceiverNode();

 private:
  void PublishCallback(const NavSatFixMsg& gps_fix);
  bool ReadParameters();

 private:
  std::string device_path_ = "/dev/ttyUSB0";
  int baud_rate_ = 115200;
  std::string frame_id_ = "gps_link";

  ros::NodeHandle nh_;
  std::shared_ptr<GpsInterface> receiver_;
  ros::Publisher pub_;
  ros::Timer pub_timer_;
  sensor_msgs::NavSatFix sat_fix_;
};
}  // namespace westonrobot

#endif /* GPS_RECEIVER_NODE_HPP */
