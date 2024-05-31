# wrp_ros mobile base

## Overview
ROS wrapper around wrp_sdk robot interfaces.

wrp_sdk uses a abstract interface to communicate with different robot platforms. This package provides a ROS wrapper around this interface to allow for easy integration with ROS systems. Due to the abstract nature of the interface, some interfaces may not be available on all robot platforms. Please refer to the [support matrix](#interface-support-matrix) below for more information.

## Support

### Robot platforms

| Robot Platform  | Manufacturer      | CAN Bitrate | Robot Type |
| --------------- | ----------------- | ----------- | ---------- |
| Scout V2        | AgileX            | 500000      | 0          |
| Scout Mini      | AgileX            | 500000      | 1          |
| Scout Mini Omni | AgileX            | 500000      | 2          |
| Ranger          | AgileX            | 500000      | 3          |
| Ranger Mini V1  | AgileX            | 500000      | 4          |
| Ranger Mini V2  | AgileX            | 500000      | 5          |
| Tracer          | AgileX            | 500000      | 6          |
| Tracer Mini     | AgileX            | 500000      | 7          |
| Hunter          | AgileX            | 500000      | 8          |
| Hunter SE       | AgileX            | 500000      | 9          |
| Bunker          | AgileX            | 500000      | 10         |
| WR Scout V2     | Weston Robot      | 1000000     | 11         |
| WR Vbot         | Weston Robot      | 1000000     | 12         |
| RobooterX       | BangBang Robotics | 500000      | 13         |

### Interface Support Matrix
Legend: Y - Supported, N - Not Supported, P - Partially Supported

| Robot Platform  | System State | Motion State | Actuator State | Odometry | Battery State | RC State | Access Control | Assisted Mode | Light Control | Motion Reset |
| --------------- | ------------ | ------------ | -------------- | -------- | ------------- | -------- | -------------- | ------------- | ------------- | ------------ |
| Scout V2        | Y            | Y            | Y              | Y        | Y             | Y        | Y              | N             | P             | N            |
| Scout Mini      | Y            | Y            | Y              | Y        | Y             | Y        | Y              | N             | P             | N            |
| Scout Mini Omni | Y            | Y            | Y              | Y        | Y             | Y        | Y              | N             | P             | N            |
| Ranger          | Y            | Y            | Y              | P        | Y             | Y        | Y              | N             | P             | N            |
| Ranger Mini V1  | Y            | Y            | Y              | P        | Y             | Y        | Y              | N             | P             | N            |
| Ranger Mini V2  | Y            | Y            | Y              | P        | Y             | Y        | Y              | N             | P             | N            |
| Tracer          | Y            | Y            | Y              | Y        | Y             | Y        | Y              | N             | P             | N            |
| Tracer Mini     | Y            | Y            | Y              | Y        | Y             | Y        | Y              | N             | P             | N            |
| Hunter          | Y            | Y            | Y              | P        | Y             | Y        | Y              | N             | P             | N            |
| Hunter SE       | Y            | Y            | Y              | P        | Y             | Y        | Y              | N             | P             | N            |
| Bunker          | Y            | Y            | Y              | Y        | Y             | Y        | Y              | N             | P             | N            |
| WR Scout V2     | Y            | Y            | Y              | Y        | Y             | Y        | Y              | N             | P             | Y            |
| WR Vbot         | Y            | Y            | Y              | Y        | Y             | Y        | Y              | N             | P             | Y            |
| RobooterX       | Y            | N            | N              | Y        | Y             | Y        | Y              | N             | N             | N            |

## Nodes

### mobile_base_node
| Published Topic    | Type                             | Description                              |
| ------------------ | -------------------------------- | ---------------------------------------- |
| `~/system_state`   | wrp_sdk_msgs::SystemState        | Outputs robot's system state             |
| `~/motion_state`   | wrp_sdk_msgs::MotionState        | Outputs robot's motion state             |
| `~/actuator_state` | wrp_sdk_msgs::ActuatorStateArray | Outputs robot's actuator states          |
| `~/odom`           | nav_msgs::Odometry               | Outputs robot's wheel odometry           |
| `~/battery_state`  | sensor_msgs::BatteryState        | Outputs robot's battery state            |
| `~/rc_state`       | wrp_sdk_msgs::RcState            | Outputs robot's rc state (if applicable) |

| Subscribed Topic | Type                 | Description              |
| ---------------- | -------------------- | ------------------------ |
| `/cmd_vel`       | geometry_msgs::Twist | Control robot's movement |

| Service                   | Type                              | Description                             |
| ------------------------- | --------------------------------- | --------------------------------------- |
| `~/access_control`        | wrp_sdk_msgs::AccessControl       | (Re)Gain or Renounce control token      |
| `~/assisted_mode_control` | wrp_sdk_msgs::AssistedModeControl | (En/Dis)able Assisted mode              |
| `~/light_control`         | wrp_sdk_msgs::LightControl        | Control robot's lights                  |
| `~/motion_reset`          | wrp_sdk_msgs::MotionReset         | Reset wheel position or odometry values |

| Parameter         | Type | Description                                                                                             |
| ----------------- | ---- | ------------------------------------------------------------------------------------------------------- |
| `robot_type`      | int  | Robot base type.<br/>Default: "0"<br/>Supported: Check [platform support table above](#robot-platforms) |
| `can_device`      | str  | Robot's CAN port.<br/>Default: "can0"                                                                   |
| `base_frame`      | str  | Base frame id.<br/>Default: "base_link"<br/>                                                            |
| `odom_frame`      | str  | Wheel odometry frame id (when publishing odometry).<br />Default: "odom"                                |
| `auto_reconnect`  | bool | Automatically attempt to gain control token.<br />Default: true                                         |
| `publish_odom_tf` | bool | If node should publish wheel odometry `tf`.<br />Default: true                                          |