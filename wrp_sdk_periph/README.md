# wrp_ros peripherals

## About

ROS wrappers around wrp_sdk peripheral drivers.

## Support

### Peripherals
| Peripheral           | Manufacturer | Interface | (Baud/Bit)rate | Node                                                |
| -------------------- | ------------ | --------- | -------------- | --------------------------------------------------- |
| Power Regulator V2.1 | Weston Robot | CAN       | 500k           | [power\_regulator\_node](#power_regulator_node)     |
| Ultrasonic Sensor    | Variable     | UART      | -              | [ultrasonic\_sensor\_node](#ultrasonic_sensor_node) |
| IMU Sensor           | Variable     | UART      | -              | [imu\_sensor\_node](#imu_sensor_node)               |
| GPS Receiver         | Variable     | UART      | -              | [gps\_receiver\_node](#gps_receiver_node)           |
| Lift Controller      | Weston Robot | UART      | -              | [lift\_controller\_node](#lift_controller_node)     |

## Nodes

### imu_sensor_node
| Published Topic | Type                  | Description                  |
| --------------- | --------------------- | ---------------------------- |
| `~/imu`         | sensor_msgs::Imu | Outputs the IMU Sensor data. |

| Parameter      | Type | Description                                                                                 |
| -------------- | ---- | ------------------------------------------------------------------------------------------- |
| `sensor_model` | str  | IMU sensor model.<br />Default: "wit"<br />Supported: "wit", "hipnuc"                       |
| `device_path`  | str  | Path to sensor port.<br />Default: "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"<br /> |
| `baud_rate`    | int  | Sensor's communication baud rate.<br />Default: "115200"                                    |
| `frame_id`     | str  | Frame id used in /imu_sensor/imu's header.<br />Default: "imu_link"                         |


### gps_receiver_node
| Published Topic | Type                        | Description                                |
| --------------- | --------------------------- | ------------------------------------------ |
| `~/fix`         | sensor_msgs::NavSatFix | Outputs the navigation satellite fix data. |

| Parameter     | Type | Description                                                                                                             |
| ------------- | ---- | ----------------------------------------------------------------------------------------------------------------------- |
| `device_path` | str  | Path to receiver port.<br />Default: "/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00"<br /> |
| `baud_rate`   | int  | Sensor's communication baud rate.<br />Default: "115200"                                                                |
| `frame_id`    | str  | Frame id used in /gps_receiver/navsat_fix's header.<br />Default: "gps_link"                                            |

### ultrasonic_sensor_node
| Published Topic       | Type                    | Description                         |
| --------------------- | ----------------------- | ----------------------------------- |
| `~/<topic_name><num>` | sensor_msgs::Range | Outputs the ultrasonic sensor data. |

| Parameter      | Type | Description                                                                                   |
| -------------- | ---- | --------------------------------------------------------------------------------------------- |
| `sensor_model` | str  | Sensor's model <br />Default: "dyp_a05"<br />Supported: "dyp_a05", "w200d"                    |
| `device_path`  | str  | Path to receiver port.<br />Default: "/dev/ttyUSB0"<br />                                     |
| `baud_rate`    | int  | Sensor's communication baud rate.<br />Default: "115200"                                      |
| `frame_id`     | str  | Frame id used in /ultrasonic_sensor_node/ultrasonic's header.<br />Default: "ultrasonic_link" |
| `topic_name`   | str  | Topic name prefix used to publish data <br />Default: "ultrasonic"                            |

### power_regulator_node
| Published Topic | Type                                         | Description                               |
| --------------- | -------------------------------------------- | ----------------------------------------- |
| `~/state`       | wrp_sdk_msgs::PowerRegulatorDeviceState | Outputs the power regulator device state. |

| Service | Type                                     | Description                               |
| ------- | ---------------------------------------- | ----------------------------------------- |
| `~/cmd` | wrp_sdk_msgs::srv::PowerRegulatorControl | (Dis)able power regulator output channels |

| Parameter     | Type | Description                                       |
| ------------- | ---- | ------------------------------------------------- |
| `device_path` | str  | Path to receiver port.<br />Default: "can0"<br /> |

### lift_controller_node
| Published Topic | Type                         | Description                                                                                          |
| --------------- | ---------------------------- | ---------------------------------------------------------------------------------------------------- |
| `~/lift_state`  | wrp_sdk_msgs::LiftState | Outputs the states of the lifts attached to the controller <br> Please check the id of the messages. |

| Service        | Type                         | Description                                                    |
| -------------- | ---------------------------- | -------------------------------------------------------------- |
| `~/lift_query` | wrp_sdk_msgs::srv::LiftQuery | Gets the state of the specific lift attached to the controller |

| Action           | Type                              | Description                                                               |
| ---------------- | --------------------------------- | ------------------------------------------------------------------------- |
| `~/lift_control` | wrp_sdk_msgs::action::LiftControl | Commands a lift attached to the controller to move to a specific position |

| Parameter       | Type   | Description                                                          |
| --------------- | ------ | -------------------------------------------------------------------- |
| `device_path`   | str    | Path to receiver port.<br />Default: "/dev/ttyUSB0"<br />            |
| `baud_rate`     | int    | Sensor's communication baud rate.<br />Default: "115200"             |
| `sampling_freq` | double | Sampling frequency of the lift controller (hz). <br /> Default: 10.0 |

#### Notes
- The lift controller node is a work in progress and speed control is not yet implemented. (speed variables are ignored)
