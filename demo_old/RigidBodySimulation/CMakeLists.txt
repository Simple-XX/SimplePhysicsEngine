cmake_minimum_required(VERSION 3.12)
project(dtkRigidBody)

set(CMAKE_CXX_STANDARD 14)

if(WIN32)
    set(freeglut_include_dir D:\\Envs\\opengl\\freeglut-3.2.1\\include)
    set(freeglut_libraries_dir D:\\Envs\\opengl\\freeglut-3.2.1\\build\\lib)
elseif(UNIX)
    set(freeglut_include_dir /usr/include/GL)
    set(freeglut_libraries_dir /usr/lib/x86_64-linux-gnu)
endif()

include_directories(${freeglut_include_dir})
link_directories(${freeglut_libraries_dir})

add_executable(dtkRigidBody main.cpp cworld.cpp cworld.h cbody.cpp cbody.h cjoint.cpp cjoint.h cpair.cpp cpair.h ctypes.cpp ctypes.h)
target_link_libraries(dtkRigidBody freeglut opengl32 glu32)