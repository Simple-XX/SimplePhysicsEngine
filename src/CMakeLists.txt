
# This file is a part of Simple-XX/SimplePhysicsEngine
# (https://github.com/Simple-XX/SimplePhysicsEngine).
#
# CMakeLists.txt for Simple-XX/SimplePhysicsEngine.

set(lib_src
        dtk/dtk.cpp
        dtk/dtkAssert.cpp
        dtk/dtkCollisionDetectBasic.cpp
        dtk/dtkCollisionDetectHierarchy.cpp
        dtk/dtkCollisionDetectHierarchyKDOPS.cpp
        dtk/dtkCollisionDetectNode.cpp
        dtk/dtkCollisionDetectNodeKDOPS.cpp
        dtk/dtkCollisionDetectPrimitive.cpp
        dtk/dtkCollisionDetectStage.cpp
        # dtk/dtkCollisionDetectStage.cpp.bak
        dtk/dtkErrorManager.cpp
        dtk/dtkGraphicsKernel.cpp
        # dtk/dtkGraphicsKernel.h.bak
        dtk/dtkIntersectTest.cpp
        dtk/dtkPhysCore.cpp
        # dtk/dtkPhysCore.cpp.bak
        dtk/dtkPhysKnotPlanner.cpp
        dtk/dtkPhysMassPoint.cpp
        dtk/dtkPhysMassSpring.cpp
        # dtk/dtkPhysMassSpring.h.bak
        dtk/dtkPhysMassSpringCollisionResponse.cpp
        dtk/dtkPhysMassSpringThread.cpp
        dtk/dtkPhysMassSpringThreadCollisionResponse.cpp
        dtk/dtkPhysParticle.cpp
        dtk/dtkPhysParticleSystem.cpp
        dtk/dtkPhysSpring.cpp
        dtk/dtkPhysTetraMassSpring.cpp
        dtk/dtkPointsReader.cpp
        dtk/dtkPointsWriter.cpp
        dtk/dtkScene.cpp
        dtk/dtkSign.cpp
        dtk/dtkStaticMeshEliminator.cpp
        dtk/dtkStaticTetraMesh.cpp
        dtk/dtkStaticTetraMeshReader.cpp
        dtk/dtkStaticTetraMeshWriter.cpp
        dtk/dtkStaticTriangleMesh.cpp
        dtk/dtkStaticTriangleMeshReader.cpp
        dtk/dtkStaticTriangleMeshWriter.cpp

        math/dtkMatrixOp.cpp

        physics/dtkJoint.cpp
        physics/dtkRigidBody.cpp
)

add_library(${PROJECT_NAME}-static STATIC
        ${lib_src}
)

target_include_directories(${PROJECT_NAME}-static PRIVATE
        $<BUILD_INTERFACE:
        ${PROJECT_SOURCE_DIR}/src/dtk/include
        ${PROJECT_SOURCE_DIR}/src/math/include
        ${PROJECT_SOURCE_DIR}/src/physics/include
        >

        $<INSTALL_INTERFACE:include/${PROJECT_NAME}-${PROJECT_VERSION}>

)

target_link_libraries(${PROJECT_NAME}-static PRIVATE
        ${DEFAULT_LINK_LIB}
        Boost::boost
        Boost::thread
        CGAL
        glfw
        glm::glm
)

# 打包
# @todo 导出静态库与动态库
packageProject(
        # 要导出的 target
        NAME ${PROJECT_NAME}-static
        # 版本
        VERSION ${PROJECT_VERSION}
        # 生成文件目录
        BINARY_DIR ${PROJECT_BINARY_DIR}
        # 头文件路径
        INCLUDE_DIR ${PROJECT_SOURCE_DIR}/src/dtk/include ${PROJECT_SOURCE_DIR}/src/math/include ${PROJECT_SOURCE_DIR}/src/physics/include
        # 与 target 的 INSTALL_INTERFACE 一致
        INCLUDE_DESTINATION include/${PROJECT_NAME}-${PROJECT_VERSION}
        # 头文件过滤
        INCLUDE_HEADER_PATTERN "*.h"
        # namespace 名称，不能包含 '::'
        NAMESPACE ${PROJECT_NAMESPACE}
        # (optional) option to disable the versioning of install destinations
        DISABLE_VERSION_SUFFIX YES
        ARCH_INDEPENDENT YES
        # 生成 CPack 变量
        CPACK YES
)
