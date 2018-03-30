add_library( utils
    config.cpp 
    vector_io.cpp 
    yes_or_no.cpp
    util.cpp
)
add_dependencies( utils
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries( utils
    ${catkin_LIBRARIES}
)
