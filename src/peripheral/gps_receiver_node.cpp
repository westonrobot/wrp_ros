#include "wrp_ros/peripheral/gps_receiver_node.hpp"

namespace westonrobot {
GpsReceiverNode::GpsReceiverNode() {
  if (!this->ReadParameters()) {
    ROS_ERROR("Could not load parameters");
    ros::shutdown();
  }

  receiver_ = std::make_shared<GpsReceiver>();

  if (!receiver_->Connect(device_path_, baud_rate_)) {
    ROS_ERROR("Failed to setup gps receiver");
    ros::shutdown();
  }

  pub_timer_ =
      nh_.createTimer(ros::Duration(publish_interval_ / 1000.0),
                      std::bind(&GpsReceiverNode::PublishCallback, this));
}

GpsReceiverNode::~GpsReceiverNode() {}

void GpsReceiverNode::PublishCallback() {}

bool GpsReceiverNode::ReadParameters() {}
}  // namespace westonrobot

int main(int argc, char** argv) {
  ros::init(argc, argv, "gps_receiver_node");
  std::cout << "gps receiver node running.\n";
  westonrobot::GpsReceiverNode gps;
  ros::spin();
  return 0;
}