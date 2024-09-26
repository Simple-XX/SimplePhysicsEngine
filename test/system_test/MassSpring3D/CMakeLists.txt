# This file is a part of Simple-XX/SimplePhysicsEngine
# (https://github.com/Simple-XX/SimplePhysicsEngine).
#
# CMakeLists.txt for Simple-XX/SimplePhysicsEngine.
cmake_minimum_required (VERSION 3.8)

project(MassSpring3D)

file(GLOB cpu_source_files
        "main.cpp"
        "ClothSimulation.cpp"
        "Shader.cpp"
        "Renderer.cpp"
        "dtkPhysMassSpringSolver.cpp"
)
file(GLOB cuda_source_files
        "*.cu"
        "cusparse_cholesky_solver.cpp"
        "cusparse_wrapper.cpp"
)

if(CUDAToolkit_FOUND)
        message(STATUS "CUDAToolkit found, compiling with CUDA support")
        enable_language(CUDA)
        
        add_executable(${PROJECT_NAME} ${cpu_source_files} ${cuda_source_files})

        # Set CUDA properties for the target
        target_include_directories(${PROJECT_NAME} PRIVATE include ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
        add_definitions(-DDTK_CUDA)

        target_link_libraries(${PROJECT_NAME} GLEW::GLEW ${CUDA_cudart_LIBRARY} ${CUDA_cusparse_LIBRARY} ${CUDA_cusolver_LIBRARY})
else()
        message(STATUS "CUDAToolkit not found, compiling without CUDA support")
        add_executable(${PROJECT_NAME} ${cpu_source_files})

        target_include_directories(${PROJECT_NAME} PRIVATE include)
        target_link_libraries(${PROJECT_NAME} GLEW::GLEW)
endif()

if (APPLE)
        target_compile_definitions(${PROJECT_NAME} PRIVATE
                GL_SILENCE_DEPRECATION
        )
endif ()
