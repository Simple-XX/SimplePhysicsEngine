cmake_minimum_required(VERSION 3.0)
project(demo3d)
set(CMAKE_CXX_STANDARD 11)

aux_source_directory(. dir_source)
file(GLOB dir_headers *.h)

include_directories({dir_headers})

if(WIN32)
    set(freeglut_include_dir "${SPE_DEPS}/freeglut/include")
    set(freeglut_libraries_dir "${SPE_DEPS}/freeglut/lib")
    set(EIGEN_PATH "${SPE_DEPS}/eigen")
    if(${VTK_OPT})
        set(vtk_include_dir D:\\Envs\\VTK\\include\\vtk-8.2)
        set(vtk_libraries_dir D:\\Envs\\VTK\\lib)
    endif()
elseif(UNIX)
    set(GLM_INCLUDE_DIR /usr/local/include/glm)
    set(freeglut_include_dir /usr/include/GL)
    set(freeglut_libraries_dir /usr/lib/x86_64-linux-gnu)
    set(EIGEN_PATH /usr/local/include/eigen3/Eigen)
    set(Boost_INCLUDE_DIR /usr/include/boost)
    set(Boost_LIBRARIES_DIR /usr/lib/x86_64-linux-gnu)
    set(DTK_INCLUDE_DIR /usr/local/include)
    set(DTK_LIBRARIES_DIR /usr/local/lib)
endif()

if(${VTK_OPT})
    add_definitions(-DUSE_VTK)
endif()

include_directories(${GLM_INCLUDE_DIR})
include_directories(${freeglut_include_dir})
link_directories(${freeglut_libraries_dir})
include_directories(${EIGEN_PATH})
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARIES_DIR}) 
include_directories(${DTK_INCLUDE_DIR})
link_directories(${DTK_LIBRARIES_DIR}) 

if(${VTK_OPT})
    include_directories(${vtk_include_dir})
    link_libraries(${vtk_libraries_dir}/*.lib)
endif()

#link

#single lib
#glfw static library include  glfw3.lib
#glfw dynamic library include  glfw3dll.lib glfw3.dll

#multi-thread lib
#glfw static library include  glfw3_mt.lib
#glfw dynamic library include  glfw3dll.lib glfw3.dll


add_executable(demo3d ${dir_source} ${dir_headers})
#target_link_libraries(FemSimulation ${GLFW_LIBRARIES} glfw3dll.lib glfw3.dll ${Boost_LIBRARIES} CGAL)

if(WIN32)
    target_link_libraries(demo3d ${GLFW_LIBRARIES}  ${Boost_LIBRARIES} freeglut opengl32 glu32)
    if(${VTK_OPT})
        target_link_libraries(demo3d ${vtk_libraries_dir}/*.lib)
    endif()
elseif(UNIX)
    target_link_libraries(demo3d ${GLFW_LIBRARIES}  ${Boost_LIBRARIES} CGAL glut OpenGL GLU dtk)
endif()
