cmake_minimum_required(VERSION 2.4.3)
set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)

#this line has to appear before 'PROJECT' in order to be able to disable incremental linking
SET(MSVC_INCREMENTAL_DEFAULT ON)

PROJECT(bullet_physics)
SET(BULLET_VERSION 2.79)
# Remap things back into the original "BULLET_PHYSICS" name of the project
SET(BULLET_PHYSICS_SOURCE_DIR ${PROJECT_SOURCE_DIR})
SET(BULLET_PHYSICS_BINARY_DIR ${PROJECT_BINARY_DIR})

IF(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
ENDIF(COMMAND cmake_policy)


find_package( catkin REQUIRED )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -g -march=native -std=c++0x -Wno-deprecated-register")


IF(NOT CMAKE_BUILD_TYPE)
    #SET(CMAKE_BUILD_TYPE "Debug")
    SET(CMAKE_BUILD_TYPE "Release")
ENDIF(NOT CMAKE_BUILD_TYPE)

#SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG")
#MESSAGE("CMAKE_CXX_FLAGS_DEBUG="+${CMAKE_CXX_FLAGS_DEBUG})

OPTION(USE_DOUBLE_PRECISION "Use double precision"	OFF)
OPTION(USE_GRAPHICAL_BENCHMARK "Use Graphical Benchmark" OFF)
OPTION(USE_MULTITHREADED_BENCHMARK "Use Multithreaded Benchmark" OFF)

IF(USE_MULTITHREADED_BENCHMARK)
    ADD_DEFINITIONS(-DUSE_PARALLEL_SOLVER_BENCHMARK -DUSE_PARALLEL_DISPATCHER_BENCHMARK)
ENDIF()


OPTION(USE_CUSTOM_VECTOR_MATH "Use custom vectormath library" OFF)

IF(USE_CUSTOM_VECTOR_MATH)
    ADD_DEFINITIONS(-DUSE_SYSTEM_VECTORMATH)
    IF(WIN32)
        SET (VECTOR_MATH_INCLUDE ${BULLET_PHYSICS_SOURCE_DIR}/src/vectormath/sse CACHE PATH "Vector Math library include path.")
    ELSE(WIN32)
        SET (VECTOR_MATH_INCLUDE ${BULLET_PHYSICS_SOURCE_DIR}/src/vectormath/scalar CACHE PATH "Vector Math library include path.")
    ENDIF(WIN32)
ENDIF(USE_CUSTOM_VECTOR_MATH)



#SET(CMAKE_EXE_LINKER_FLAGS_INIT    "/STACK:10000000 /INCREMENTAL:NO")
#SET(CMAKE_EXE_LINKER_FLAGS    "/STACK:10000000 /INCREMENTAL:NO")






OPTION(BUILD_MINICL_OPENCL_DEMOS "Build OpenCL demos for MiniCL (Generic CPU)" OFF)

FIND_PATH(INTEL_OPENCL_INCLUDES CL/cl.h)
FIND_PATH(INTEL_OPENCL_ICD_CFG intelocl64.icd /etc/OpenCL/vendors)
FIND_LIBRARY(INTEL_OPENCL_LIBRARIES OpenCL PATH /usr/lib64)
IF(INTEL_OPENCL_INCLUDES AND INTEL_OPENCL_LIBRARIES AND INTEL_OPENCL_ICD_CFG)
        MESSAGE("INTEL OPENCL SDK FOUND")
        MESSAGE(${INTEL_OPENCL_LIBRARIES})
        OPTION(BUILD_INTEL_OPENCL_DEMOS "Build OpenCL demos for Intel (CPU)"        ON)
ELSE()
        MESSAGE("INTEL OPENCL NOT FOUND")
        OPTION(BUILD_INTEL_OPENCL_DEMOS "Build OpenCL demos for Intel (CPU)"        OFF)
ENDIF()

FIND_PATH(NVIDIA_OPENCL_INCLUDES CL/cl.h)
FIND_PATH(NVIDIA_OPENCL_ICD_CFG nvidia.icd /etc/OpenCL/vendors)
FIND_LIBRARY(NVIDIA_OPENCL_LIBRARIES OpenCL PATH /usr/lib64 /usr/local/lib)
IF(NVIDIA_OPENCL_INCLUDES AND NVIDIA_OPENCL_LIBRARIES AND NVIDIA_OPENCL_ICD_CFG)
            MESSAGE("NVidia OPENCL FOUND")
    MESSAGE(${NVIDIA_OPENCL_LIBRARIES})
    OPTION(BUILD_NVIDIA_OPENCL_DEMOS "Build OpenCL demos for NVidia (GPU)"	ON)
ELSE()
    MESSAGE("NVidia OPENCL NOT FOUND")
    OPTION(BUILD_NVIDIA_OPENCL_DEMOS "Build OpenCL demos for NVidia (GPU)"	OFF)
ENDIF()


OPTION(BUILD_CPU_DEMOS "Build original Bullet CPU demos"        OFF)





OPTION(INTERNAL_UPDATE_SERIALIZATION_STRUCTURES "Internal update serialization structures" OFF)
IF(INTERNAL_UPDATE_SERIALIZATION_STRUCTURES)
    ADD_DEFINITIONS( -DBT_INTERNAL_UPDATE_SERIALIZATION_STRUCTURES)
ENDIF(INTERNAL_UPDATE_SERIALIZATION_STRUCTURES)

IF(USE_DOUBLE_PRECISION)
    ADD_DEFINITIONS( -DBT_USE_DOUBLE_PRECISION)
    SET( BULLET_DOUBLE_DEF "-DBT_USE_DOUBLE_PRECISION")
ENDIF(USE_DOUBLE_PRECISION)

IF(USE_GRAPHICAL_BENCHMARK)
    ADD_DEFINITIONS( -DUSE_GRAPHICAL_BENCHMARK)
ENDIF(USE_GRAPHICAL_BENCHMARK)





# This is the shortcut to finding GLU, GLUT and OpenGL if they are properly installed on your system
# This should be the case.

FIND_PACKAGE(OpenGL)
IF(OPENGL_FOUND)
ELSE (OPENGL_FOUND)
    SET(OPENGL_gl_LIBRARY opengl32)
    SET(OPENGL_glu_LIBRARY glu32)
ENDIF(OPENGL_FOUND)





OPTION(BUILD_DEMOS "Set when you want to build the demos" OFF)
IF(BUILD_DEMOS)
    IF(EXISTS ${BULLET_PHYSICS_SOURCE_DIR}/Demos AND IS_DIRECTORY ${BULLET_PHYSICS_SOURCE_DIR}/Demos)
        SUBDIRS(Demos)
    ENDIF()
ENDIF(BUILD_DEMOS)




OPTION(BUILD_EXTRAS "Set when you want to build the extras" ON)
IF(BUILD_EXTRAS)
    SUBDIRS(Extras)
ENDIF(BUILD_EXTRAS)



SUBDIRS(src)




IF("${CMAKE_GENERATOR}" MATCHES "Unix Makefiles")
    OPTION(INSTALL_LIBS "Set when you want to install libraries" OFF)
ELSE()
    IF(APPLE AND FRAMEWORK)
        OPTION(INSTALL_LIBS "Set when you want to install libraries" OFF)
    ELSE()
#by default, don't enable the 'INSTALL' option for Xcode and MSVC projectfiles
        OPTION(INSTALL_LIBS "Set when you want to install libraries" OFF)
    ENDIF()
ENDIF()




IF(INSTALL_LIBS)
    SET (LIB_SUFFIX "" CACHE STRING "Define suffix of directory name (32/64)" )
    SET (LIB_DESTINATION "${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}" CACHE STRING "Library directory name")
    ## the following are directories where stuff will be installed to
    SET(INCLUDE_INSTALL_DIR      "${CMAKE_INSTALL_PREFIX}/include/bullet/" CACHE PATH "The subdirectory to the header prefix")
    SET(PKGCONFIG_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}/pkgconfig/" CACHE STRING "Base directory for pkgconfig files")
    IF(NOT WIN32)
        CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/bullet.pc.cmake ${CMAKE_CURRENT_BINARY_DIR}/bullet.pc @ONLY)
    INSTALL(
        FILES
        ${CMAKE_CURRENT_BINARY_DIR}/bullet.pc
        DESTINATION
        ${PKGCONFIG_INSTALL_PREFIX})
    ENDIF(NOT WIN32)
ENDIF(INSTALL_LIBS)




#INSTALL of other files requires CMake 2.6
IF(${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.5)
    OPTION(INSTALL_EXTRA_LIBS "Set when you want extra libraries installed" OFF)
ENDIF(${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.5)

OPTION(BUILD_UNIT_TESTS "Build Unit Tests"	OFF)
IF(BUILD_UNIT_TESTS)
    SUBDIRS(UnitTests)
ENDIF()





catkin_package(
    INCLUDE_DIRS    src
                    Extras
                    Extras/HACD
    LIBRARIES       LinearMath
                    BulletCollision
                    BulletDynamics
                    BulletSoftBody
                    BulletMultiThreaded
                    HACD
                    BulletFileLoader
)
