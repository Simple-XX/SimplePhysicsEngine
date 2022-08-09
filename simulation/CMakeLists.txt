cmake_minimum_required(VERSION 2.6)

project(sph)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

set(freeglut_include_dir D:/Envs/opengl/freeglut-3.2.1/include)
set(freeglut_libraries_dir D:/Envs/opengl/freeglut-3.2.1/build/lib)
set(EIGEN_PATH D:/Envs/Eigen/eigen-3.3.8)

include_directories(
	"${freeglut_include_dir}"
	"${EIGEN_PATH}"
	"${PROJECT_SOURCE_DIR}/include"
)

link_directories(${freeglut_libraries_dir})

file(GLOB_RECURSE sph_FILES
	"${PROJECT_SOURCE_DIR}/include/*.h"
	"${PROJECT_SOURCE_DIR}/src/*.cpp"
)

add_executable(sph ${sph_FILES})