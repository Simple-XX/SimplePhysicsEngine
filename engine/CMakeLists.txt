
# This file is a part of Simple-XX/SimplePhysicsEngine
# (https://github.com/Simple-XX/SimplePhysicsEngine).
#
# CMakeLists.txt for Simple-XX/SimplePhysicsEngine.

find_package(CGAL)

project(dtk)

option(GROUP_BY_EXPLORER ON)

# aux_source_directory(. dir_source)
file(GLOB_RECURSE dir_source *.cpp)
file(GLOB_RECURSE dir_headers *.h)

set(all_files ${dir_source} ${dir_headers})

include_directories(${SPE_ROOT_DIR})

if(WIN32)
    message(STATUS "Compiling on Windows os")
    set(Boost_INCLUDE_DIR C:\\local\\boost_1_79_0)
    set(Boost_LIBRARIES_DIR C:\\local\\boost_1_79_0\\lib64-msvc-14.2)
    set(GLM_INCLUDE_DIR "${SPE_DEPS}/glm")
elseif(UNIX)
    message(STATUS "Compiling on UNIX-like os")
    set(Boost_INCLUDE_DIR /usr/include/boost)
    set(Boost_LIBRARIES_DIR /usr/lib/x86_64-linux-gnu)
    set(GLM_INCLUDE_DIR /usr/local/include/glm)
endif()

include_directories(${Boost_INCLUDE_DIR})
# link_directories(${Boost_LIBRARIES_DIR})
include_directories(${GLM_INCLUDE_DIR}) 

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

# link
# link_libraries(Boost)
link_libraries(CGAL::CGAL)
add_library(dtk STATIC ${all_files}) 
# target_link_libraries(dtk Boost)

#[[install(
    TARGETS dtk
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

# install
install(TARGETS dtk DESTINATION ${DTK_INSTALL_LIBDIR})
install(FILES ${dir_headers} DESTINATION ${DTK_INSTALL_INCLUDEDIR})]]
