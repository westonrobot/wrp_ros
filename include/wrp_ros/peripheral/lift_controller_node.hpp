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

#include <wrp_ros/LiftQuery.h>
#include <wrp_ros/LiftStatus.h>
#include <wrp_ros/LiftControlAction.h>
#include "wrp_sdk/peripheral/lift_controller.hpp"
#include "wrp_sdk/interface/lift_interface.hpp"

namespace westonrobot {
class LiftControllerNode {
 public:
  LiftControllerNode();
  ~LiftControllerNode();

 private:
  int baud_rate_ = 115200;
  std::string device_path_ = "/dev/ttyUSB0";

  ros::NodeHandle nh_;
  ros::Publisher lift_status_pub_;
  ros::ServiceServer query_server_;
  actionlib::SimpleActionServer<wrp_ros::LiftControlAction> lift_control_server_;

  westonrobot::LiftController lift_controller_;

  void PublishLiftState(void);
  static void ExitSignalHandler(int sig);
  void LiftControllerCallback(const wrp_ros::LiftControlGoalConstPtr& goal);
  bool QueryCallback(wrp_ros::LiftQuery::Request& req,
                     wrp_ros::LiftQuery::Response& res);
  bool ReadParameters(void);
};
}  // namespace westonrobot
#endif /* LIFT_CONTROLLER_NODE */