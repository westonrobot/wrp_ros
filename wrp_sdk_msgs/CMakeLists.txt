cmake_minimum_required(VERSION 3.0.2)
project(wrp_sdk_msgs)

# # Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  geometry_msgs
  std_msgs
  actionlib_msgs
)

add_message_files(
  FILES

  # mobile base
  "ActuatorState.msg"
  "ActuatorStateArray.msg"
  "DriverState.msg"
  "LightControlType.msg"
  "MotionCommand.msg"
  "MotionState.msg"
  "MotorState.msg"
  "RangeData.msg"
  "RangeDataArray.msg"
  "RcState.msg"
  "SystemState.msg"

  # peripheral
  "PowerRegulatorChannelState.msg"
  "PowerRegulatorDeviceState.msg"
  "LiftState.msg"
)

# # Generate services in the 'srv' folder
add_service_files(
  FILES

  # mobile base
  "AccessControl.srv"
  "AssistedModeControl.srv"
  "LightControl.srv"
  "MotionReset.srv"

  # peripheral
  "PowerRegulatorControl.srv"
  "LiftQuery.srv"
)

# Generate actions in the 'action' folder
add_action_files(
  FILES

  # peripheral
  "LiftControl.action"
)

# # Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS std_msgs actionlib_msgs geometry_msgs message_runtime)

include_directories(
  ${catkin_INCLUDE_DIRS}
)