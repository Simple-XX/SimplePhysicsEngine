
# This file is a part of Simple-XX/SimplePhysicsEngine
# (https://github.com/Simple-XX/SimplePhysicsEngine).
#
# CMakeLists.txt for Simple-XX/SimplePhysicsEngine.

project(demo2d)

file(GLOB_RECURSE demo2d_FILES
        "${PROJECT_SOURCE_DIR}/include/*.h"
        "${PROJECT_SOURCE_DIR}/src/*.cpp"
)

add_executable(${PROJECT_NAME}
        ${demo2d_FILES}
)

target_include_directories(${PROJECT_NAME} PRIVATE
        include
        ../
        ../engine/dtk
        ../engine/math
        ../engine/physics
)

target_link_libraries(${PROJECT_NAME} PRIVATE
        dtk
        Boost::headers
        Boost::dynamic_linking
        CGAL
        Eigen
        glut
        GLUT::GLUT
        glfw
        glm
        OpenGL::GL
        OpenGL::GLU 
)

message("GLUT_glut_LIBRARY: ${GLUT_glut_LIBRARY}")
