cmake_minimum_required(VERSION 2.8.3)
project(james_helper_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
)


# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Float32Array.msg
)

# Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Example.srv
# )

# Generate actions in the 'action' folder
# add_action_files(
#    DIRECTORY action
#    FILES
#    Example.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
)


## CATKIN_DEPENDS: catkin_packages dependent projects also need
catkin_package(
  CATKIN_DEPENDS roscpp rospy message_runtime
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)
