cmake_minimum_required(VERSION 2.6)

project(simualtion)
set(CMAKE_CXX_STANDARD 11)

find_package(CGAL)

set(freeglut_include_dir D:\\Envs\\opengl\\freeglut-3.2.1\\include)
set(freeglut_libraries_dir D:\\Envs\\opengl\\freeglut-3.2.1\\build\\lib)
set(EIGEN_PATH D:\\Envs\\Eigen\\eigen-3.3.8)
set(GLM_INCLUDE_DIR D:\\Envs\\glm)

include_directories(
	"${freeglut_include_dir}"
	"${EIGEN_PATH}"
	"${PROJECT_SOURCE_DIR}/include"
	"${PROJECT_SOURCE_DIR}/../src/"
	"${GLM_INCLUDE_DIR}"
)

link_directories(${freeglut_libraries_dir})

file(GLOB_RECURSE simualtion_FILES
	"${PROJECT_SOURCE_DIR}/include/*.h"
	"${PROJECT_SOURCE_DIR}/src/*.cpp"
	"${PROJECT_SOURCE_DIR}/../src/dtkRigidBody.h"
	"${PROJECT_SOURCE_DIR}/../src/dtkRigidBody.cpp"
	"${PROJECT_SOURCE_DIR}/../src/dtkTx.h"
	"${PROJECT_SOURCE_DIR}/../src/dtkTxOP.h"
	"${PROJECT_SOURCE_DIR}/../src/dtkMatrix.h"
	"${PROJECT_SOURCE_DIR}/../src/dtkMatrixOP.h"
	"${PROJECT_SOURCE_DIR}/../src/dtkMatrixOp.cpp"
	"${PROJECT_SOURCE_DIR}/../src/dtkJoint.h"
	"${PROJECT_SOURCE_DIR}/../src/dtkJoint.cpp"
	
)

add_executable(simualtion ${simualtion_FILES})

target_link_libraries(simualtion CGAL)