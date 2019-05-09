cmake_minimum_required(VERSION 2.8.3)
project(actionlib_enhanced)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  message_runtime
  rospy
)

catkin_python_setup()

add_action_files(
  FILES
  BasicCom.action
)
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  LIBRARIES actionlib_enhanced
  CATKIN_DEPENDS actionlib actionlib_msgs message_runtime std_msgs
  DEPENDS Boost
)


###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############

install(PROGRAMS
  examples/client.py
  examples/server.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples
)
