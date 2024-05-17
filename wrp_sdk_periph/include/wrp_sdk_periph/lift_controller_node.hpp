/**
 * @file lift_controller_node.hpp
 * @brief This file contains the implementation of a simple action and service
 *        server for lift controller node. 
 *
 * @date 06-02-2024
 *
 * @copyright Copyright (c) 2024 Weston Robot Pte. Ltd.
 */
#ifndef LIFT_CONTROLLER_NODE
#define LIFT_CONTROLLER_NODE

#include <signal.h>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include "wrp_sdk/peripheral/lift_controller.hpp"

#include <wrp_sdk_msgs/LiftQuery.h>
#include <wrp_sdk_msgs/LiftState.h>
#include <wrp_sdk_msgs/LiftControlAction.h>

namespace westonrobot {
class LiftControllerNode {
 public:
  LiftControllerNode();
  ~LiftControllerNode();

 private:
  int baud_rate_ = 115200;
  std::string device_path_ = "/dev/ttyUSB0";

  ros::NodeHandle nh_;
  ros::Publisher lift_state_pub_;
  ros::ServiceServer query_server_;
  actionlib::SimpleActionServer<wrp_sdk_msgs::LiftControlAction> lift_control_server_;

  wrp_sdk_msgs::LiftState lift_state_;
  LiftController lift_controller_;

  void PublishLiftState(void);
  static void ExitSignalHandler(int sig);
  void LiftControllerCallback(const wrp_sdk_msgs::LiftControlGoalConstPtr& goal);
  bool QueryCallback(wrp_sdk_msgs::LiftQuery::Request& req,
                     wrp_sdk_msgs::LiftQuery::Response& res);
  bool ReadParameters(void);
};
}  // namespace westonrobot
#endif /* LIFT_CONTROLLER_NODE */