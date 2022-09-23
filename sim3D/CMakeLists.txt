cmake_minimum_required(VERSION 3.0)
project(FemSimulation)
set(CMAKE_CXX_STANDARD 11)

aux_source_directory(. dir_source)
file(GLOB dir_headers *.h)

include_directories({dir_headers})

if(WIN32)
    set(freeglut_include_dir D:\\Envs\\opengl\\freeglut-3.2.1\\include)
    set(freeglut_libraries_dir D:\\Envs\\opengl\\freeglut-3.2.1\\build\\lib)
    set(EIGEN_PATH D:\\Envs\\Eigen\\eigen-3.3.8)

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

#include_directories(${GLFW_INCLUDE_DIR})
#include_directories(${GLAD_INCLUDE_DIR})
include_directories(${GLM_INCLUDE_DIR})
#link_directories(${GLFW_LIBRARIES_DIR}) 
include_directories(${freeglut_include_dir})
link_directories(${freeglut_libraries_dir})
include_directories(${EIGEN_PATH})
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARIES_DIR}) 
include_directories(${DTK_INCLUDE_DIR})
link_directories(${DTK_LIBRARIES_DIR}) 

#link

#single lib
#glfw static library include  glfw3.lib
#glfw dynamic library include  glfw3dll.lib glfw3.dll

#multi-thread lib
#glfw static library include  glfw3_mt.lib
#glfw dynamic library include  glfw3dll.lib glfw3.dll


add_executable(FemSimulation ${dir_source} ${dir_headers})
#target_link_libraries(FemSimulation ${GLFW_LIBRARIES} glfw3dll.lib glfw3.dll ${Boost_LIBRARIES} CGAL)

if(WIN32)
    target_link_libraries(FemSimulation ${GLFW_LIBRARIES}  ${Boost_LIBRARIES} freeglut opengl32 glu32)
elseif(UNIX)
    target_link_libraries(FemSimulation ${GLFW_LIBRARIES}  ${Boost_LIBRARIES} CGAL glut OpenGL GLU dtk)
endif()
