cmake_minimum_required(VERSION 2.8.3)
project(smmap)

set(CATKIN_PACKAGES
    actionlib
    arc_utilities
    geometry_msgs
    kinematics_toolbox
    image_transport
    cv_bridge
    ${PROJECT_NAME}_msgs
    ${PROJECT_NAME}_experiment_params
)

## Add our local cmake modules to the list that can be found
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGES})

## System dependencies are found with CMake's conventions
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
find_package(GUROBI REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Messages defined in smmap_msgs package

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS    include
#     LIBRARIES       ${PROJECT_NAME}
    CATKIN_DEPENDS  ${CATKIN_PACKAGES}
    DEPENDS         Eigen3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
    SYSTEM
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${GUROBI_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -O2 -g -flto -Wall -Wextra -Wpedantic -Wconversion")

unset(USE_OPEN_MP CACHE)
option(USE_OPEN_MP "Set to ON if we want to use OpenMP" ON)
if(USE_OPEN_MP)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -DUSE_OPEN_MP")
    message(STATUS "Enabling OpenMP for SMMAP library")
else(USE_OPEN_MP)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unknown-pragmas")
    message(WARNING "Disabling OpenMP for SMMAP library")
endif(USE_OPEN_MP)


## Library file for the whole project
FILE(GLOB hpp_SRC "include/${PROJECT_NAME}/*.hpp")
FILE(GLOB h_SRC "include/${PROJECT_NAME}/*.h")
FILE(GLOB cpp_SRC "src/*.cpp")

add_library(${PROJECT_NAME}_KFMANDB ${hpp_SRC} ${h_SRC} ${cpp_SRC})
add_dependencies(${PROJECT_NAME}_KFMANDB ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_KFMANDB ${catkin_LIBRARIES} ${GUROBI_LIBRARIES})
set_target_properties(${PROJECT_NAME}_KFMANDB PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} -DKFMANDB_BANDIT")

add_library(${PROJECT_NAME}_KFMANB ${hpp_SRC} ${h_SRC} ${cpp_SRC})
add_dependencies(${PROJECT_NAME}_KFMANB ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_KFMANB ${catkin_LIBRARIES} ${GUROBI_LIBRARIES})
set_target_properties(${PROJECT_NAME}_KFMANB PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} -DKFMANB_BANDIT")

add_library(${PROJECT_NAME}_UCB ${hpp_SRC} ${h_SRC} ${cpp_SRC})
add_dependencies(${PROJECT_NAME}_UCB ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_UCB ${catkin_LIBRARIES} ${GUROBI_LIBRARIES})
set_target_properties(${PROJECT_NAME}_UCB PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} -DUCB_BANDIT")

## Simulator tests
add_executable(${PROJECT_NAME}_test_node_KFMANDB nodes/${PROJECT_NAME}_test_node.cpp)
add_dependencies(${PROJECT_NAME}_test_node_KFMANDB ${PROJECT_NAME}_KFMANDB ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test_node_KFMANDB ${PROJECT_NAME}_KFMANDB ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_test_node_KFMANDB PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} -DKFMANDB_BANDIT")

add_executable(${PROJECT_NAME}_test_node_KFMANB nodes/${PROJECT_NAME}_test_node.cpp)
add_dependencies(${PROJECT_NAME}_test_node_KFMANB ${PROJECT_NAME}_KFMANB ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test_node_KFMANB ${PROJECT_NAME}_KFMANB ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_test_node_KFMANB PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} -DKFMANB_BANDIT")

add_executable(${PROJECT_NAME}_test_node_UCB nodes/${PROJECT_NAME}_test_node.cpp)
add_dependencies(${PROJECT_NAME}_test_node_UCB ${PROJECT_NAME}_UCB ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test_node_UCB ${PROJECT_NAME}_UCB ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_test_node_UCB PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} -DUCB_BANDIT")

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#     scripts/my_python_script
#     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS smmap smmap_node
#     ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#     DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#     FILES_MATCHING PATTERN "*.h"
#     PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#     # myfile1
#     # myfile2
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_smmap.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#     target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
