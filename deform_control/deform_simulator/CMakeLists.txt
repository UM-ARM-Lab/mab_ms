cmake_minimum_required( VERSION 2.8.3 )
project( deform_simulator )

set( CATKIN_PACKAGES
    actionlib
    arc_utilities
    geometry_msgs
    visualization_msgs
    roscpp
    osg_bullet
    haptics
    bullet_helpers
    smmap_msgs
    smmap_experiment_params
)

## Find catkin macros and libraries
find_package( catkin REQUIRED COMPONENTS ${CATKIN_PACKAGES} )

## System dependencies are found with CMake's conventions
find_package( cmake_modules REQUIRED )
find_package( Boost REQUIRED COMPONENTS
    system
    filesystem
    program_options
    thread
)

find_package( Eigen3 REQUIRED )
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

################################################
## Declare ROS messages, services and actions ##
################################################

# Declared in the smmap_msgs package

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS    include
#    LIBRARIES       ${PROJECT_NAME}
    CATKIN_DEPENDS  ${CATKIN_PACKAGES}
    DEPENDS         Eigen3
                    Boost
)


###########
## Build ##
###########

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O2 -std=c++0x" )

include_directories( include SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
)

set( ${PROJECT_NAME}_LINK_TARGETS
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_subdirectory( src )
add_subdirectory( nodes )
