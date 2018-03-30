cmake_minimum_required( VERSION 2.8.3 )
project( osg_works )

set( CATKIN_PACKAGES
    OpenSceneGraph
)

find_package( catkin REQUIRED COMPONENTS ${CATKIN_PACKAGES} )

catkin_package(
    INCLUDE_DIRS    ${CMAKE_CURRENT_SOURCE_DIR}
    LIBRARIES       ${PROJECT_NAME}
    CATKIN_DEPENDS  ${CATKIN_PACKAGES}
)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -g -std=c++0x -march=native")

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}
    SYSTEM
    ${catkin_INCLUDE_DIRS}
)

set( OSGWORKS_SOURCES
    osgwQuery/QueryStats.cpp
    osgwQuery/QueryObject.cpp
    osgwQuery/QueryBenchmarks.cpp
    osgwQuery/QueryBin.cpp
#    osgwQuery/QueryDrawCB.cpp
    osgwQuery/QueryUtils.cpp
    osgwQuery/QueryAPI.cpp
    osgwQuery/QueryComputation.cpp
    osgwControls/SliderControl.cpp
    osgwTools/InsertRemove.cpp
    osgwTools/CountStateSets.cpp
    osgwTools/Uniqifier.cpp
    osgwTools/CallbackSupport.cpp
    osgwTools/CollapseLOD.cpp
    osgwTools/ForceFlattenTransforms.cpp
    osgwTools/CountsVisitor.cpp
    osgwTools/ReducerOp.cpp
    osgwTools/PluginLoader.cpp
    osgwTools/AbsoluteModelTransform.cpp
    osgwTools/RemoveProgram.cpp
    osgwTools/Transform.cpp
    osgwTools/FBOUtils.cpp
    osgwTools/RemoveData.cpp
    osgwTools/NodeUtils.cpp
    osgwTools/FindNamedNode.cpp
    osgwTools/DataLoader.cpp
    osgwTools/TransparencyUtils.cpp
    osgwTools/LODCreationNodeVisitor.cpp
    osgwTools/GeometryModifier.cpp
    osgwTools/ScreenCapture.cpp
    osgwTools/ShortEdgeOp.cpp
    osgwTools/GeometryOperation.cpp
    osgwTools/NodePathUtils.cpp
    osgwTools/TangentSpaceOp.cpp
    osgwTools/StateSetUtils.cpp
    osgwTools/ReadFile.cpp
    osgwTools/ParallelVisitor.cpp
    osgwTools/SimplifierOp.cpp
    osgwTools/Quat.cpp
    osgwTools/Version.cpp
    osgwTools/DecimatorOp.cpp
    osgwTools/RemoveLOD.cpp
    osgwTools/CameraConfigObject.cpp
    osgwTools/RefID.cpp
    osgwTools/Shapes.cpp
    osgwTools/PrimitiveSetConversion.cpp
)

add_library( ${PROJECT_NAME}
    ${OSGWORKS_SOURCES}
)
add_dependencies( ${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries( ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)
