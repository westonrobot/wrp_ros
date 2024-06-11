# wrp_ros

## About 

This package contains a minimal wrapper around wrp_sdk and provides a ROS interface to hardware platforms from Weston Robot. Please first check if your device and environment are supported by the SDK and this ROS package before proceeding:

***SDK Documentation and Sample Code can be found [here](https://github.com/westonrobot/wrp_sdk)***

**Supported Environments**:

* Architecture: x86_64/arm64
* OS: Ubuntu 20.04
* ROS: Noetic
  
**Supported Robots**:

* Refer to [mobile_base](./wrp_sdk_robot/README.md) for more details.

**Supported Peripherals**:

* Power Regulator V2.1
* Ultrasonic Sensor
* Hipnuc and WitMotion IMU
* NMEA-compatible GPS Receiver
* Lift Controller

More details can be found in the README inside individual src sub-folders

  * [Mobile Base Node](./wrp_sdk_robot/README.md)
  * [Peripheral Nodes](./wrp_sdk_periph/README.md)

## Getting the package

### Install dependencies

* wrp_sdk >= **v1.3.0** ... < **v1.5** : please follow setup instructions from [here](https://github.com/westonrobot/wrp_sdk/)

* Peripheral driver dependencies (optional if you are not using peripherals): 
    ```bash
    $ sudo apt-get install -y software-properties-common 
    $ sudo add-apt-repository ppa:lely/ppa
    $ sudo apt-get install -y pkg-config liblely-coapp-dev liblely-co-tools
    ```

### Build the package

* Clone the packages into a catkin workspace and compile/source.  
(the following instructions assume your catkin workspace is at: ~/catkin_ws/src)

    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    git clone https://github.com/westonrobot/wrp_ros.git
    cd ..
    catkin_make
    . devel/setup.bash
    ```

## Setup CAN-To-USB adapter
**Only needed if you are using a CAN robot or peripheral**
 
1. Enable gs_usb kernel module
    ```
    $ sudo modprobe gs_usb
    ```
2. Bringup can device

    **NOTE**: Adjust CAN bitrate accordingly. Refer to the tables in the [mobile_base README](./wrp_sdk_robot/README.md) and [peripheral README](./wrp_sdk_periph/README.md) for the correct bitrate.
   ```
   $ sudo ip link set can0 up type can bitrate <bitrate>
   $ sudo ip link set can0 txqueuelen 1000
   ```
3. If no error occured during the previous steps, you should be able to see the can device now by using command
   ```
   $ ifconfig -a
   ```
4. Install and use can-utils to test the hardware
    ```
    $ sudo apt install can-utils
    ```
5. Testing command
    ```
    # receiving data from can0
    $ candump can0
    # send data to can0
    $ cansend can0 001#1122334455667788
    ```

Scripts are provided [here](./scripts) for convenience. You can run "./setup_can2usb.bash" for the first-time setup and run "./bringup_can2usb_1m.bash" to bring up the device each time you unplug and re-plug the adapter.

## Example Usage

You can find more information about robot base control from [this page](https://docs.westonrobot.net/getting_started/basics/robot_base_control.html).

**_You may need to change run time parameters by editing the corresponding launch file_**

1. Mobile Base Node (and [variants](./wrp_sdk_robot/launch/mobile_base))

    ```bash
    roslaunch wrp_sdk_robot mobile_base.launch.py
    ```

2. Power Regulator Node

    ```bash
    roslaunch wrp_sdk_periph power_regulator.launch.py
    ```

3. GPS Receiver Node

    ```bash
    roslaunch wrp_sdk_periph gps_receiver_launch.py 
    ```

4. IMU Sensor Node

    ```bash
    roslaunch wrp_sdk_periph imu_sensor_launch.py 
    ```

5. Ultrasonic Sensor Node

    ```bash
    roslaunch wrp_sdk_periph ultrasonic_sensor.launch.py
    ```