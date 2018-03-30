cmake_minimum_required( VERSION 2.8.3 )
project( haptics )

find_package( catkin REQUIRED )
find_package( cmake_modules )
find_package( Eigen3 )
set( Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR} )

catkin_package(
    INCLUDE_DIRS    include
    LIBRARIES       ${PROJECT_NAME}
    DEPENDS         Eigen3
)
include_directories( include
    SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)
add_library( ${PROJECT_NAME}
    src/UDPSocket.cpp
    src/thread_socket_interface.cpp
)
