
# This file is a part of Simple-XX/SimplePhysicsEngine
# (https://github.com/Simple-XX/SimplePhysicsEngine).
#
# CMakeLists.txt for Simple-XX/SimplePhysicsEngine.

project(demo3d)

add_executable(demo3d
        main.cpp
        dtkFemSimulation.cpp
        dtkScene.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
        include
)

target_link_libraries(${PROJECT_NAME} PRIVATE
        Eigen
        GLUT::GLUT
        glm
        OpenGL::GL
        OpenGL::GLU
)
