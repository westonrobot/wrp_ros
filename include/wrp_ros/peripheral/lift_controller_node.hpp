/**
 * @file lift_controller_node.hpp
 * @brief This file contains the implementation of a simple action and service
 *        server for lift controller node. 
 *
 * @date 06-02-2024
 *
 * @copyright Copyright (c) 2024 Weston Robot Pte. Ltd.
 */

#ifndef LIFT_SERVER_NODE
#define LIFT_SERVER_NODE

#include <ros/ros.h>
#include <wrp_ros/LiftQuery.h>
#include <wrp_ros/LiftControlAction.h>
#include <actionlib/server/simple_action_server.h>

#include "wrp_sdk/peripheral/lift_controller.hpp"
#include "wrp_sdk/interface/lift_interface.hpp"

namespace westonrobot {
class LiftControllerNode {
 public:
  LiftControllerNode();
  ~LiftControllerNode();

  bool QueryCallback(wrp_ros::LiftQuery::Request& req,
                     wrp_ros::LiftQuery::Response& res);

 private:
  int baud_rate_ = 115200;
  std::string device_path_ = "/dev/ttyUSB0";

  ros::NodeHandle nh_;
  ros::ServiceServer query_server_;
  actionlib::SimpleActionServer<wrp_ros::LiftControlAction> lift_control_server_;
  westonrobot::LiftController lift_controller_;

  void LiftControllerCallback(const wrp_ros::LiftControlGoalConstPtr& goal);
  bool ReadParameters();
};
}  // namespace westonrobot
#endif /* LIFT_SERVER_NODE */