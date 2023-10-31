
# This file is a part of Simple-XX/SimplePhysicsEngine
# (https://github.com/Simple-XX/SimplePhysicsEngine).
#
# CMakeLists.txt for Simple-XX/SimplePhysicsEngine.

project(demo2d)

add_executable(${PROJECT_NAME}
        Constants.cpp
        dtkCollisionPair.cpp
        dtkFemSimulation.cpp
        dtkMesh.cpp
        dtkObject.cpp
        dtkScene.cpp
        Grid.cpp
        Main.cpp
        Particle.cpp
        SPHSolver.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
        include
)

target_link_libraries(${PROJECT_NAME} PRIVATE
        Eigen
        GLUT::GLUT
        glm
        OpenGL::GL
)
