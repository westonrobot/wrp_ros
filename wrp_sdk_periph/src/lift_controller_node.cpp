/**
 * @file  lift_controller_node.cpp
 * @brief This file contains the implementation of a simple action and service
 *        server for lift controller node.
 *
 * @date  09-01-2024
 *
 * @copyright Copyright (c) 2024 Weston Robot Pte. Ltd.
 */
#include "wrp_sdk_periph/lift_controller_node.hpp"

namespace westonrobot {
LiftControllerNode::LiftControllerNode()
    : lift_controller_(),
      lift_control_server_(
          nh_, "/lift_controller/goal",
          [this](const wrp_sdk_msgs::LiftControlGoalConstPtr& goal) {
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

  lift_state_pub_ =
      nh_.advertise<wrp_sdk_msgs::LiftState>("/lift_controller/lift_state", 10);

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
    const wrp_sdk_msgs::LiftControlGoalConstPtr& goal) {
  wrp_sdk_msgs::LiftControlFeedback feedback;
  wrp_sdk_msgs::LiftControlResult result;

  if (goal->id != lift_state_.LIFT_HORIZONTAL &&
      goal->id != lift_state_.LIFT_VERTICAL) {
    ROS_ERROR("Invalid id. Valid id are 0 (horizontal) and 1 (vertical)");
    result.id = goal->id;
    lift_control_server_.setAborted(result, "Invalid id");
    return;
  }

  lift_controller_.SendCommandToLift(goal->position, goal->speed, goal->id);
  
  LiftState state;
  static uint8_t prevPos = state.position;
  do {
    state = lift_controller_.GetLiftState(goal->id);
    if (state.position != prevPos) {
      prevPos = state.position;

      feedback.id = state.id;
      feedback.position = state.position;
      feedback.speed = state.speed;
      lift_control_server_.publishFeedback(feedback);
    }
  } while (state.position != (goal->position));

  result.id = state.id;
  result.position = state.position;
  result.speed = state.speed;

  lift_control_server_.setSucceeded(result);
}

bool LiftControllerNode::QueryCallback(wrp_sdk_msgs::LiftQuery::Request& req,
                                       wrp_sdk_msgs::LiftQuery::Response& res) {
  if (req.id != lift_state_.LIFT_HORIZONTAL &&
      req.id != lift_state_.LIFT_VERTICAL) {
    ROS_ERROR("Invalid id. Valid id are 0 (horizontal) and 1 (vertical)");
    return false;
  }

  LiftState state = lift_controller_.GetLiftState(req.id);
  res.position = state.position;
  res.speed = state.speed;
  return true;
}

void LiftControllerNode::PublishLiftState() {
  signal(SIGINT, LiftControllerNode::ExitSignalHandler);
  while (ros::ok) {
    for (uint8_t id = lift_state_.LIFT_HORIZONTAL;
         id <= lift_state_.LIFT_VERTICAL; ++id) {
      LiftState lift_state = lift_controller_.GetLiftState(id);

      lift_state_.id = id;
      lift_state_.position = lift_state.position;
      lift_state_pub_.publish(lift_state_);
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
