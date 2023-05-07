cmake_minimum_required(VERSION 2.6)

project(demo2d)
set(CMAKE_CXX_STANDARD 11)

# find_package(CGAL)

set(freeglut_include_dir D:\\Envs\\opengl\\freeglut-3.2.1\\include)
set(freeglut_libraries_dir D:\\Envs\\opengl\\freeglut-3.2.1\\build\\lib)
set(EIGEN_PATH D:\\Envs\\Eigen\\eigen-3.3.8)
set(GLM_INCLUDE_DIR D:\\Envs\\glm)

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