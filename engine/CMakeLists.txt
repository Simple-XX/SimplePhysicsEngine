
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

# # 打包
# # @todo 导出静态库与动态库
# packageProject(
#         # 要导出的 target
#         NAME dtk
#         # 版本
#         VERSION ${PROJECT_VERSION}
#         # 生成文件目录
#         BINARY_DIR ${PROJECT_BINARY_DIR}
#         # 头文件路径
#         INCLUDE_DIR ${PROJECT_SOURCE_DIR}/src/include
#         # 与 target 的 INSTALL_INTERFACE 一致
#         INCLUDE_DESTINATION include/${PROJECT_NAME}-${PROJECT_VERSION}
#         # 头文件过滤
#         INCLUDE_HEADER_PATTERN "*.h"
#         # namespace 名称，不能包含 '::'
#         NAMESPACE ${PROJECT_NAMESPACE}
#         # (optional) option to disable the versioning of install destinations
#         DISABLE_VERSION_SUFFIX YES
#         ARCH_INDEPENDENT YES
#         # 生成 CPack 变量
#         CPACK YES
# )
