# This file is a part of Simple-XX/SimplePhysicsEngine
# (https://github.com/Simple-XX/SimplePhysicsEngine).
#
# CMakeLists.txt for Simple-XX/SimplePhysicsEngine.

project(MassSpring3D)

add_executable(${PROJECT_NAME}
        main.cpp
        ClothSimulation.cpp
        Shader.cpp
        Renderer.cpp
        dtkPhysMassSpringSolver.cpp
        step.cu
)

target_include_directories(${PROJECT_NAME} PRIVATE
        include
)

# find_package(CUDA REQUIRED)
find_package(CUDAToolkit)

target_link_libraries(${PROJECT_NAME} GLEW::GLEW CUDA::cudart CUDA::cusparse)

if (APPLE)
        target_compile_definitions(${PROJECT_NAME} PRIVATE
                GL_SILENCE_DEPRECATION
        )
endif ()
