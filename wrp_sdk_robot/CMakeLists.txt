cmake_minimum_required(VERSION 3.0.2)
project(wrp_sdk_robot)

# # Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

string(REPLACE ":" ";" CMAKE_PREFIX_PATH "$ENV{CMAKE_PREFIX_PATH}")
list(APPEND CMAKE_PREFIX_PATH "/opt/weston_robot/lib/cmake")

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  wrp_sdk_msgs
)

find_package(wrp_sdk 1.3 REQUIRED)

# ##################################
# # catkin specific configuration ##
# ##################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs tf2 tf2_ros wrp_sdk_msgs
  DEPENDS wrp_sdk
)

# ##########
# # Build ##
# ##########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mobile_base_node src/mobile_base_node.cpp)
target_link_libraries(mobile_base_node ${catkin_LIBRARIES} westonrobot::wrp_sdk_robot)
add_dependencies(mobile_base_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)