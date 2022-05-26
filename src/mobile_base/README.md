# wrp_ros mobile base

## About

ROS wrappers around wrp_sdk mobile base interface.

## Nodes

### mobile_base_node
| Published Topic   | Type                        | Description                     |
| ----------------- | --------------------------- | ------------------------------- |
| `/system_state`   | wrp_ros::SystemState        | Outputs robot's system state    |
| `/motion_state`   | wrp_ros::MotionState        | Outputs robot's motion state    |
| `/actuator_state` | wrp_ros::ActuatorStateArray | Outputs robot's actuator states |
| `/odom`           | nav_msgs::msg::Odometry     | Outputs robot's wheel odometry  |
| `/battery_state`  | sensor_msgs::BatteryState   | Outputs robot's battery state   |

| Subscribed Topic | Type                      | Description              |
| ---------------- | ------------------------- | ------------------------ |
| `/cmd_vel`       | geometry_msgs::msg::Twist | Control robot's movement |

| Service                  | Type                         | Description                             |
| ------------------------ | ---------------------------- | --------------------------------------- |
| `/access_control`        | wrp_ros::AccessControl       | (Re)Gain or Renounce control token      |
| `/assisted_mode_control` | wrp_ros::AssistedModeControl | (En/Dis)able Assisted mode              |
| `/light_control`         | wrp_ros::LightControl        | Control robot's lights                  |
| `/motion_reset`          | wrp_ros::MotionReset         | Reset wheel position or odometry values |

| Parameter         | Type | Description                                                                                         |
| ----------------- | ---- | --------------------------------------------------------------------------------------------------- |
| `robot_base_type` | str  | Robot base type.<br/>Default: "weston"<br/>Supported: "weston", "agilex" & "vbot"                   |
| `can_device`      | str  | Robot's CAN port.<br/>Default: "can0"                                                               |
| `base_frame`      | str  | Base frame id.<br/>Default: "base_link"<br/>                                                        |
| `odom_frame`      | str  | Odometry frame id.<br />Default: "odom"                                                             |
| `auto_reconnect`  | bool | Automatically attempt to gain control token.<br />Default: true                                     |
| `motion_type`     | str  | Robot's motion type. <br/>Default: "skid_steer"<br/>Supported: "skid_steer", "omni", "differential" |
