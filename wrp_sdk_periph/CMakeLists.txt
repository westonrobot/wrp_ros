cmake_minimum_required(VERSION 3.0.2)
project(wrp_sdk_periph)

# # Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

string(REPLACE ":" ";" CMAKE_PREFIX_PATH "$ENV{CMAKE_PREFIX_PATH}")
list(APPEND CMAKE_PREFIX_PATH "/opt/weston_robot/lib/cmake")

find_package(PkgConfig REQUIRED)
pkg_check_modules(LelyIO QUIET IMPORTED_TARGET liblely-io2)
pkg_check_modules(LelyCOAPP QUIET IMPORTED_TARGET liblely-coapp)

set(BUILD_CANOPEN_SUPPORT OFF)

if(LelyIO_FOUND AND LelyCOAPP_FOUND)
  message(STATUS "Lely-core libraries found, peripheral module will be included")
  set(BUILD_CANOPEN_SUPPORT ON)
else()
  message(WARNING "CANopen-related dependencies are not found, ROS nodes for peripherals will not be built. Run the following commands to install the missing libraries:
    $ sudo apt-get install -y software-properties-common 
    $ sudo add-apt-repository ppa:lely/ppa && sudo apt-get update
    $ sudo apt-get install -y pkg-config liblely-coapp-dev liblely-co-tools")
endif()

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  wrp_sdk_msgs
  actionlib
)

find_package(wrp_sdk 1.3 REQUIRED)

# ##################################
# # catkin specific configuration ##
# ##################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp actionlib wrp_sdk_msgs
  DEPENDS wrp_sdk
)

# ##########
# # Build ##
# ##########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(lift_controller_node src/lift_controller_node.cpp)
target_link_libraries(lift_controller_node westonrobot::wrp_sdk_peripheral ${catkin_LIBRARIES})
add_dependencies(lift_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ultrasonic_sensor_node src/ultrasonic_sensor_node.cpp)
target_link_libraries(ultrasonic_sensor_node westonrobot::wrp_sdk_peripheral ${catkin_LIBRARIES})
add_dependencies(ultrasonic_sensor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# canopen device support
if(BUILD_CANOPEN_SUPPORT)
  add_executable(gps_receiver_node src/gps_receiver_node.cpp)
  target_link_libraries(gps_receiver_node westonrobot::wrp_sdk ${catkin_LIBRARIES})
  add_dependencies(gps_receiver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  add_executable(imu_sensor_node src/imu_sensor_node.cpp)
  target_link_libraries(imu_sensor_node westonrobot::wrp_sdk ${catkin_LIBRARIES})
  add_dependencies(imu_sensor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  add_executable(power_regulator_node src/power_regulator_node.cpp)
  target_link_libraries(power_regulator_node westonrobot::wrp_sdk ${catkin_LIBRARIES})
  add_dependencies(power_regulator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif()

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)