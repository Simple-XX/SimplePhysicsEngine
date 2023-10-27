
# This file is a part of Simple-XX/SimplePhysicsEngine
# (https://github.com/Simple-XX/SimplePhysicsEngine).
#
# CMakeLists.txt for Simple-XX/SimplePhysicsEngine.

project(dtk)

option(GROUP_BY_EXPLORER ON)

# aux_source_directory(. dir_source)
file(GLOB_RECURSE dir_source *.cpp)
file(GLOB_RECURSE dir_headers *.h)

set(all_files ${dir_source} ${dir_headers})

foreach(fileItem ${all_files})       
    # Get the directory of the source file
    get_filename_component(PARENT_DIR "${fileItem}" DIRECTORY)
    # Remove common directory prefix to make the group
    string(REPLACE "${CMAKE_CURRENT_SOURCE_DIR}" "" GROUP "${PARENT_DIR}")
    # Make sure we are using windows slashes
    string(REPLACE "/" "\\" GROUP "${GROUP}")
    # Group into "Source Files" and "Header Files"
    set(GROUP "${GROUP}")
    source_group("${GROUP}" FILES "${fileItem}")
endforeach()

add_library(dtk STATIC ${all_files})

target_include_directories(dtk PRIVATE 
    ./
    math
    dtk
    physics
) 

target_link_libraries(dtk PRIVATE
    Boost 
    CGAL
    GLUT
    glfw
    glm
)

#[[install(
    TARGETS dtk
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

# install
install(TARGETS dtk DESTINATION ${DTK_INSTALL_LIBDIR})
install(FILES ${dir_headers} DESTINATION ${DTK_INSTALL_INCLUDEDIR})]]
