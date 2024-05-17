# wrp_sdk_periph peripherals

## About

ROS wrappers around wrp_sdk peripheral drivers.

## Nodes

### imu_sensor_node
| Published Topic | Type             | Description                  |
| --------------- | ---------------- | ---------------------------- |
| `/imu`          | sensor_msgs::Imu | Outputs the IMU Sensor data. |

| Parameter      | Type | Description                                                                                 |
| -------------- | ---- | ------------------------------------------------------------------------------------------- |
| `sensor_model` | str  | IMU sensor model.<br />Default: "wit"<br /> Supported: "wit", "hipnuc"                      |
| `device_path`  | str  | Path to sensor port.<br />Default: "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"<br /> |
| `baud_rate`    | int  | Sensor's communication baud rate.<br />Default: "115200"                                    |
| `frame_id`     | str  | Frame id used in /imu_sensor/imu's header.<br />Default: "imu_link"                         |


### gps_receiver_node
| Published Topic | Type                   | Description                                |
| --------------- | ---------------------- | ------------------------------------------ |
| `/fix`          | sensor_msgs::NavSatFix | Outputs the navigation satellite fix data. |

| Parameter     | Type | Description                                                                                                             |
| ------------- | ---- | ----------------------------------------------------------------------------------------------------------------------- |
| `device_path` | str  | Path to receiver port.<br />Default: "/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00"<br /> |
| `baud_rate`   | int  | Sensor's communication baud rate.<br />Default: "115200"                                                                |
| `frame_id`    | str  | Frame id used in /gps_receiver/navsat_fix's header.<br />Default: "gps_link"                                            |

### ultrasonic_sensor_node
| Published Topic    | Type               | Description                         |
| ------------------ | ------------------ | ----------------------------------- |
| `/ultrasonic<num>` | sensor_msgs::Range | Outputs the ultrasonic sensor data. |

| Parameter      | Type   | Description                                                                                   |
| -------------- | ------ | --------------------------------------------------------------------------------------------- |
| `sensor_model` | string | Sensor's model.<br />Default: "dyp_a05"<br /> Supported: "dyp_a05", "w200d"                   |
| `device_path`  | str    | Path to sensor port.<br />Default: "/dev/ttyUSB0"<br />                                       |
| `baud_rate`    | int    | Sensor's communication baud rate.<br />Default: "115200"                                      |
| `frame_id`     | str    | Frame id used in /ultrasonic_sensor_node/ultrasonic's header.<br />Default: "ultrasonic_link" |
| `topic name`   | str    | Topic used in /ultrasonic_sensor_node/ultrasonic's header.<br />Default: "ultrasonic"         |

### power_regulator_node

| Published Topic          | Type                               | Description                             |
| ------------------------ | ---------------------------------- | --------------------------------------- |
| `/power_regulator/state` | wrp_sdk_periph::PowerRegulatorDeviceState | Outputs the power regulator state data. |

| Service                | Type                           | Description                              |
| ---------------------- | ------------------------------ | ---------------------------------------- |
| `/power_regulator/cmd` | wrp_sdk_periph::PowerRegulatorControl | Control power regulator output channels. |

| Parameter     | Type   | Description                                                  |
| ------------- | ------ | ------------------------------------------------------------ |
| `device_path` | string | Path to power regulator can port.<br />Default: "can0"<br /> |

### lift_controller_node

| Published Topic                | Type                | Description                                |
| ------------------------------ | ------------------- | ------------------------------------------ |
| `/lift_controller/lift_state` | wrp_sdk_periph::LiftState | Outputs the lift id and position. |

| Action                  | Type                       | Description                  |
| ----------------------- | -------------------------- | ---------------------------- |
| `/lift_controller/goal` | wrp_sdk_periph::LiftControlAction | Provides a goal to the lift. |

| Service                        | Type               | Description                              |
| ------------------------------ | ------------------ | ---------------------------------------- |
| `/lift_controller/query_state` | wrp_sdk_periph::LiftQuery | Query for state of the provided lift id. |

| Parameter     | Type   | Description                                                      |
| ------------- | ------ | ---------------------------------------------------------------- |
| `device_path` | string | Path to lift controller port.<br />Default: "/dev/ttyUSB0"<br /> |
| `baud_rate`   | int    | Lift server's communication baud rate.<br />Default: "115200"    |
