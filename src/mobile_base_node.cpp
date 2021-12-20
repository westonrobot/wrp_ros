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
  ros::init(argc, argv, "scout");
  ros::NodeHandle node(""), private_node("~");

  std::string port_name;
  private_node.param<std::string>("port_name", port_name, std::string("vcan0"));

  std::string robot_type;
  private_node.param<std::string>("robot_type", robot_type,
                                  std::string("vbot"));

  std::string base_frame_name;
  private_node.param<std::string>("base_frame", base_frame_name,
                                  std::string("base_link"));

  std::string odom_frame_name;
  std::string odom_topic_name;
  private_node.param<std::string>("odom_frame", odom_frame_name,
                                  std::string("odom"));
  private_node.param<std::string>("odom_topic_name", odom_topic_name,
                                  std::string("odom"));

  RobotBaseType base_type;
  if (robot_type == "weston") {
    base_type = RobotBaseType::kWeston;
  } else if (robot_type == "agilex") {
    base_type = RobotBaseType::kAgilex;
  } else if (robot_type == "vbot") {
    base_type = RobotBaseType::kAgilex;
  } else if (robot_type == "unitree_a1") {
    base_type = RobotBaseType::kUnitreeA1;
  } else {
    std::cout << "Unknown robot type " << robot_type << std::endl;
    return -1;
  }

  MobileBaseRos base(&node, port_name, base_type);

  base.SetBaseFrame(base_frame_name);
  base.SetOdomFrame(odom_frame_name);
  base.SetOdomTopicName(odom_topic_name);

  base.SetAutoReconnect(true);

  base.Run(50);

  return 0;
}