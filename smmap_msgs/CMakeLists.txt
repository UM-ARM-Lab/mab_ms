cmake_minimum_required( VERSION 2.8.3 )
project( smmap_msgs )

set( CATKIN_PACKAGES
  actionlib_msgs
  geometry_msgs
  std_msgs
)
## Find catkin macros and libraries
find_package( catkin REQUIRED COMPONENTS ${CATKIN_PACKAGES} message_generation )

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    ConfidenceStamped.msg
    VectorPose.msg
    SimulatorFeedback.msg
    PointCloud.msg
)

## Generate services in the 'srv' folder
add_service_files(
    FILES
    ExecuteGripperMovement.srv
    GetGripperNames.srv
    GetGripperAttachedNodeIndices.srv
    GetGripperCollisionReport.srv
    GetGripperPose.srv
    GetPointSet.srv
    GetMirrorLine.srv
    GetFreeSpaceGraph.srv
)

## Generate actions in the 'action' folder
add_action_files(
    FILES
    TestGrippersPoses.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS    include
    LIBRARIES       ${PROJECT_NAME}
    CATKIN_DEPENDS  ${CATKIN_PACKAGES}
                    message_runtime
)

###########
## Build ##
###########

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -g -O2 -Wall -Wextra -Wpedantic -Wconversion" )

## Let QtCreator find these files...

include_directories( include
    SYSTEM
    ${catkin_INCLUDE_DIRS}
)

add_library( ${PROJECT_NAME}
    include/${PROJECT_NAME}/messages.h
    src/messages.cpp
)
add_dependencies( ${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencpp
)
