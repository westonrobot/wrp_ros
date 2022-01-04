/*
 * mobile_base_node.cpp
 *
 * Created on: Dec 17, 2021 09:59
 * Description:
 *
 * Copyright (c) 2021 Weston Robot Pte. Ltd.
 */

#include <ros/ros.h>

#include "wrp_ros/mobile_base/mobile_base_ros.hpp"

using namespace westonrobot;

int main(int argc, char **argv) {
  // setup ROS node
  ros::init(argc, argv, "mobile_base_node");
  ros::NodeHandle node("~");

  MobileBaseRos base(&node);

  base.Run(50);

  return 0;
}