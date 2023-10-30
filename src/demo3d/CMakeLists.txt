
# This file is a part of Simple-XX/SimplePhysicsEngine
# (https://github.com/Simple-XX/SimplePhysicsEngine).
#
# CMakeLists.txt for Simple-XX/SimplePhysicsEngine.

project(demo3d)

#single lib
#glfw static library include  glfw3.lib
#glfw dynamic library include  glfw3dll.lib glfw3.dll

#multi-thread lib
#glfw static library include  glfw3_mt.lib
#glfw dynamic library include  glfw3dll.lib glfw3.dll

add_executable(demo3d
        main.cpp
        dtkFemSimulation.cpp
        dtkScene.cpp
)

target_include_directories(demo3d PRIVATE
        ./
        ../
        ../engine/dtk
        ../engine/math
        ../engine/physics
)

target_link_libraries(demo3d PRIVATE
        dtk
        Boost::headers
        Boost::dynamic_linking
        CGAL
        GLUT::GLUT
        glfw
        glm
        Eigen
        OpenGL::GL
        glut
        GLU
)

target_compile_definitions(demo3d PRIVATE
#        $<VTK_OPT:USE_VTK>
)

#target_link_libraries(FemSimulation ${GLFW_LIBRARIES} glfw3dll.lib glfw3.dll ${Boost_LIBRARIES} CGAL)

#if (WIN32)
#    target_link_libraries(demo3d ${GLFW_LIBRARIES} ${Boost_LIBRARIES} freeglut opengl32 glu32)
#    if (${VTK_OPT})
#        target_link_libraries(demo3d ${vtk_libraries_dir}/*.lib)
#    endif ()
#elseif (UNIX)
#    target_link_libraries(demo3d ${GLFW_LIBRARIES} ${Boost_LIBRARIES} CGAL glut OpenGL GLU dtk)
#endif ()
