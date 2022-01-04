# wrp_ros

## About

This package contains a minimal wrapper around wrp_sdk
More details in the individual src sub-folders
  1. [Peripheral Nodes](./src/peripheral)
  2. [Mobile Base Node](./src/mobile_base)

## Basic Usage

1. Clone the packages into a colcon workspace and compile/source.  
(the following instructions assume your catkin workspace is at: ~/catkin_ws/src)

    ```bash
    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws/src
    $ git clone https://gitlab.com/westonrobot/ros/wrp_ros.git
    $ cd ..
    $ catkin_make
    $ source devel/setup.bash
    ```

1. Launch ROS nodes  
    **_Change run time parameters by editing the corresponding launch file_**

    1. GPS Receiver Node

        ```bash
        roslaunch wrp_ros gps_receiver.launch 
        ```

    2. IMU Sensor Node

        ```bash
        roslaunch wrp_ros imu_sensor.launch 
        ```

    3. Mobile Base Node (and [variants](./launch/mobile_base))

        ```bash
        roslaunch wrp_ros scout_base.launch
        ```
