uint8 CONTROL_SOURCE_RC = 0
uint8 CONTROL_SOURCE_CAN = 1
uint8 CONTROL_SOURCE_ETHERNET = 2

geometry_msgs/Vector3 desired_linear
geometry_msgs/Vector3 desired_angular
int32 source
bool collision_detected
bool assisted_mode_enabled