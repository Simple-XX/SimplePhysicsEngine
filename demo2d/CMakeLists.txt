
# This file is a part of Simple-XX/SimplePhysicsEngine
# (https://github.com/Simple-XX/SimplePhysicsEngine).
#
# CMakeLists.txt for Simple-XX/SimplePhysicsEngine.

cmake_minimum_required(VERSION 2.6)

project(demo2d)
set(CMAKE_CXX_STANDARD 11)

set(freeglut_include_dir "${SPE_DEPS}/freeglut/include")
set(freeglut_libraries_dir "${SPE_DEPS}/freeglut/lib")
set(EIGEN_PATH "${SPE_DEPS}/eigen")
set(GLM_INCLUDE_DIR "${SPE_DEPS}/glm")

include_directories(
	"${freeglut_include_dir}"
	"${EIGEN_PATH}"
	"${PROJECT_SOURCE_DIR}/include"
	"${SPE_ROOT_DIR}"
	"${GLM_INCLUDE_DIR}"
)

link_directories(${freeglut_libraries_dir})

file(GLOB_RECURSE demo2d_FILES
	"${PROJECT_SOURCE_DIR}/include/*.h"
	"${PROJECT_SOURCE_DIR}/src/*.cpp"
)

add_executable(${PROJECT_NAME} ${demo2d_FILES})
target_link_libraries(${PROJECT_NAME} dtk)