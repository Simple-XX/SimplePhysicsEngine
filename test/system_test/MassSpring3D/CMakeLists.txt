# This file is a part of Simple-XX/SimplePhysicsEngine
# (https://github.com/Simple-XX/SimplePhysicsEngine).
#
# CMakeLists.txt for Simple-XX/SimplePhysicsEngine.

project(MassSpring3D)

add_executable(${PROJECT_NAME}
        main.cpp
        ClothSimulation.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
        include
)

if (APPLE)
        target_compile_definitions(${PROJECT_NAME} PRIVATE
                GL_SILENCE_DEPRECATION
        )
endif ()
