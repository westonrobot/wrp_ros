# wrp_ros

## About

This package contains a minimal wrapper around wrp_sdk.
More details in the individual src sub-folders
  1. [Peripheral Nodes](./src/peripheral)
  2. [Mobile Base Node](./src/mobile_base)

## Dependencies

1. wrp_sdk: v1.0.0 
follow instructions from [here](https://github.com/westonrobot/wrp_sdk/tree/sample-v1.0.0)

## Setup CAN-To-USB adapter
 
1. Enable gs_usb kernel module
    ```
    $ sudo modprobe gs_usb
    ```
2. Bringup can device
   ```
   $ sudo ip link set can0 up type can bitrate 1000000
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

Scripts are provided [here](./scripts) for easy setup. You can run "./setup_can2usb.bash" for the first-time setup and run "./bringup_can2usb_1m.bash" to bring up the device each time you unplug and re-plug the adapter.

## Basic Usage

1. Clone the packages into a colcon workspace and compile/source.  
(the following instructions assume your catkin workspace is at: ~/catkin_ws/src)

    ```bash
    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/westonrobot/wrp_ros.git
    $ cd ..
    $ catkin_make
    $ source devel/setup.bash
    ```

2. Launch ROS nodes  
    **_Change run time parameters by editing the corresponding launch file_**

    1. GPS Receiver Node

        ```bash
        roslaunch wrp_ros gps_receiver.launch 
        ```

    2. IMU Sensor Node

        ```bash
        roslaunch wrp_ros imu_sensor.launch 
        ```

    3. Ultrasonic Sensor Node

        ```bash
        roslaunch wrp_ros ultrasonic_sensor.launch 
        ```

    4. Mobile Base Node (and [variants](./launch/mobile_base))

        ```bash
        roslaunch wrp_ros scout_base.launch
        ```
