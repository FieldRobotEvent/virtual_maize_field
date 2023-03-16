cmake_minimum_required(VERSION 3.0.2)
project(virtual_maize_field)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslaunch
)

# Generate the python library
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package(
 CATKIN_DEPENDS rospy roslaunch
)

###########
## Build ##
###########

install(DIRECTORY launch models worlds config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
