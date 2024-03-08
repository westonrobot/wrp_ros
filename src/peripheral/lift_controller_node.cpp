/**
 * @file  lift_controller_node.cpp
 * @brief This file contains the implementation of a simple action and service
 *        server for lift controller node.
 *
 * @date  09-01-2024
 *
 * @copyright Copyright (c) 2024 Weston Robot Pte. Ltd.
 */
#include "wrp_ros/peripheral/lift_controller_node.hpp"

namespace westonrobot {
LiftControllerNode::LiftControllerNode()
    : lift_controller_(),
      lift_control_server_(
          nh_, "/lift_controller/goal",
          [this](const wrp_ros::LiftControlGoalConstPtr& goal) {
            this->LiftControllerCallback(goal);
          },
          false) {
  if (!ReadParameters()) {
    ROS_ERROR("Could not load parameters");
    ros::shutdown();
  }
  lift_controller_.Connect(device_path_, baud_rate_);
  if (!lift_controller_.IsOkay()) {
    std::cerr << "Failed to connect to lift controller" << std::endl;
    ros::shutdown();
  }

  lift_status_pub_ =
      nh_.advertise<wrp_ros::LiftStatus>("/lift_controller/lift_status", 10);

  query_server_ = nh_.advertiseService(
      "/lift_controller/query_state", &LiftControllerNode::QueryCallback, this);

  lift_control_server_.start();

  LiftControllerNode::PublishLiftState();
}

LiftControllerNode::~LiftControllerNode() {
  lift_controller_.Disconnect();
  ros::shutdown();
}

void LiftControllerNode::LiftControllerCallback(
    const wrp_ros::LiftControlGoalConstPtr& goal) {
  wrp_ros::LiftControlFeedback feedback;
  wrp_ros::LiftControlResult result;

  if (goal->position == 0) {
    lift_controller_.ResetState(goal->id);
  } else {
    lift_controller_.SendCommandToLift(goal->position, goal->speed, goal->id);
  }

  LiftState state;
  static uint8_t prevPos = state.position;  // To reduce redundant updates
  do {
    state = lift_controller_.GetLiftState(goal->id);
    if (state.position != prevPos) {
      prevPos = state.position;

      feedback.id = state.id;
      feedback.position = state.position;
      feedback.speed = state.speed;
      lift_control_server_.publishFeedback(feedback);
    }
  } while (state.position >= (goal->position) + 2 ||
           state.position <= (goal->position) - 2);

  result.id = state.id;
  result.position = state.position;
  result.speed = state.speed;

  lift_control_server_.setSucceeded(result);
}

bool LiftControllerNode::QueryCallback(wrp_ros::LiftQuery::Request& req,
                                       wrp_ros::LiftQuery::Response& res) {
  LiftState state = lift_controller_.GetLiftState(req.id);
  res.position = state.position;
  res.speed = state.speed;
  return true;
}

void LiftControllerNode::PublishLiftState() {
  signal(SIGINT, LiftControllerNode::ExitSignalHandler);
  wrp_ros::LiftStatus lift_status_msg;

  while (ros::ok) {
    for (uint8_t orientation = lift_status_msg.LIFT_HORIZONTAL;
         orientation <= lift_status_msg.LIFT_VERTICAL; ++orientation) {
      LiftState lift_state = lift_controller_.GetLiftState(orientation);

      lift_status_msg.orientation = orientation;
      lift_status_msg.position = lift_state.position;
      lift_status_pub_.publish(lift_status_msg);
    }
    ros::spinOnce();
    ros::Rate(10).sleep();
  }
}

void LiftControllerNode::ExitSignalHandler(int sig) {
  ROS_INFO("Ctrl+C detected, shutting down...");
  ros::shutdown();
  exit(0);
}

bool LiftControllerNode::ReadParameters() {
  nh_.getParam("device_path", device_path_);
  nh_.getParam("baud_rate", baud_rate_);

  ROS_INFO(
      "Successfully loaded the following parameters: \nDevice path: %s\nBaud "
      "rate: %d\n",
      device_path_.c_str(), baud_rate_);

  return true;
}
}  // namespace westonrobot

int main(int argc, char** argv) {
  // Initialise ROS node
  ros::init(argc, argv, "lift_controller_node");
  westonrobot::LiftControllerNode Server;
  ros::spin();
  return 0;
}
