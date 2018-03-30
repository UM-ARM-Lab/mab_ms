#include_directories( SYSTEM
#    ${BULLET_DIR}/Extras
#    ${BULLET_DIR}/Extras/HACD
#    ${BULLETSIM_SOURCE_DIR}/lib/haptics
#    ${BULLETSIM_SOURCE_DIR}/src
#)
#target_link_libraries(simulation 
#    utils 
#    haptics
#    osgBullet
#    ${OSG_LIBS} 
#    ${BULLET_LIBS} 
#)

add_library(simulation 
    environment.cpp 
    basicobjects.cpp 
    simplescene.cpp 
    softbodies.cpp 
    rope.cpp 
    plotting.cpp 
    convexdecomp.cpp 
    recording.cpp 
    config_bullet.cpp 
    config_scene.cpp 
    config_viewer.cpp 
    bullet_io.cpp
)
add_dependencies( simulation
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries( simulation
    ${Boost_LIBRARIES} 
    ${catkin_LIBRARIES}
    utils
)
