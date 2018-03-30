cmake_minimum_required( VERSION 2.8.3 )
project( bullet_helpers )

## Find catkin macros and libraries
set( CATKIN_PACKAGES
    osg_bullet
    roscpp
    geometry_msgs
    visualization_msgs
    arc_utilities
)

find_package( catkin REQUIRED COMPONENTS ${CATKIN_PACKAGES} )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS    include
#    LIBRARIES       ${PROJECT_NAME}
    CATKIN_DEPENDS  ${CATKIN_PACKAGES}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include SYSTEM
  ${catkin_INCLUDE_DIRS}
)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O2 -flto -march=native -std=c++0x -Wall -Wextra -Wpedantic -Werror" )

## Declare a C++ library
add_library( ${PROJECT_NAME}
    include/${PROJECT_NAME}/bullet_internal_conversions.hpp
    src/bullet_internal_conversions.cpp
    include/${PROJECT_NAME}/bullet_eigen_conversions.hpp
    src/bullet_eigen_conversions.cpp
    include/${PROJECT_NAME}/bullet_ros_conversions.hpp
    src/bullet_ros_conversions.cpp
    include/${PROJECT_NAME}/bullet_math_helpers.hpp
    src/bullet_math_helpers.cpp
    include/${PROJECT_NAME}/bullet_pretty_print.hpp
    src/bullet_pretty_print.cpp
)
add_dependencies( ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} )

#############
## Install ##
#############

## Mark libraries for installation
#install(TARGETS ${PROJECT_NAME}
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN ".svn" EXCLUDE
    PATTERN "tags" EXCLUDE
)
