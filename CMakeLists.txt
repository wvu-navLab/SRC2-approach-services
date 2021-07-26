################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(src2_approach_services)

add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  actionlib_msgs
  genmsg
  actionlib
  tf2
  message_runtime
  message_generation
  message_filters
  cv_bridge
  stereo_msgs
)


################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

add_service_files(
  FILES


  ApproachChargingStation.srv
  ApproachBin.srv
  ApproachExcavator.srv
  FindBin.srv

)
generate_messages(
	DEPENDENCIES
  actionlib_msgs
	std_msgs
	geometry_msgs
	sensor_msgs
  stereo_msgs
)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs nav_msgs tf message_generation message_runtime
)

################################################################################
# Build
################################################################################


################################################################################
# Install
################################################################################
install(PROGRAMS src/approach_bin.py src/approach_charging_station.py src/approach_excavator.py src/base_approach_class.py src/find_bin.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

##########################
#######EXECUTABLES########
##########################
