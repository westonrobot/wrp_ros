#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include "wrp_sdk/peripheral/gps_receiver.hpp"

namespace westonrobot {
namespace wrp_ros {
class GpsReceiverNode {
 public:
  GpsReceiverNode();
  ~GpsReceiverNode();

 private:
  void PublishCallback(const NavSatFix& gps_fix);
  bool ReadParameters();

 private:
  ros::NodeHandle nh_;
  std::shared_ptr<GpsReceiver> receiver_;
  ros::Publisher pub_;
  ros::Timer pub_timer_;
  sensor_msgs::NavSatFix sat_fix_;
  std::string device_path_ = "/dev/ttyUSB0";
  int publish_interval_ = 500;
  int baud_rate_ = 115200;
  std::string frame_id_ = "gps";
};
}  // namespace wrp_ros
}  // namespace westonrobot