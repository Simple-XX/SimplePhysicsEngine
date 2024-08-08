
# This file is a part of Simple-XX/SimplePhysicsEngine
# (https://github.com/Simple-XX/SimplePhysicsEngine).
#
# CMakeLists.txt for Simple-XX/SimplePhysicsEngine.

set(lib_src
        collision_detect/dtkCollisionDetectBasic.cpp
        collision_detect/dtkCollisionDetectHierarchy.cpp
        collision_detect/dtkCollisionDetectHierarchyKDOPS.cpp
        collision_detect/dtkCollisionDetectNode.cpp
        collision_detect/dtkCollisionDetectNodeKDOPS.cpp
        collision_detect/dtkCollisionDetectPrimitive.cpp
        collision_detect/dtkCollisionDetectStage.cpp
        dtk.cpp
        dtkErrorManager.cpp
        dtkGraphicsKernel.cpp
        dtkIntersectTest.cpp
        dtkPhysCore.cpp
        dtkPhysKnotPlanner.cpp
        dtkPhysMassPoint.cpp
        dtkPhysMassSpring.cpp
        dtkPhysMassSpringCollisionResponse.cpp
        dtkPhysMassSpringThread.cpp
        dtkPhysMassSpringThreadCollisionResponse.cpp
        dtkPhysParticle.cpp
        dtkPhysParticleSystem.cpp
        dtkPhysSpring.cpp
        dtkPhysTetraMassSpring.cpp
        dtkPointsReader.cpp
        dtkPointsWriter.cpp
        dtkScene.cpp
        dtkSign.cpp
        dtkStaticMeshEliminator.cpp
        dtkStaticTetraMesh.cpp
        dtkStaticTetraMeshReader.cpp
        dtkStaticTetraMeshWriter.cpp
        dtkStaticTriangleMesh.cpp
        dtkStaticTriangleMeshReader.cpp
        dtkStaticTriangleMeshWriter.cpp

        math/dtkMatrixOp.cpp

        physics/dtkJoint.cpp
        physics/dtkRigidBody.cpp
)

link_libraries(
        ${DEFAULT_LINK_LIB}
        Boost::headers
        Boost::thread
        CGAL
        $<BUILD_INTERFACE:glfw>
)

add_library(${PROJECT_NAME}-shared SHARED
        ${lib_src}
)

target_compile_definitions(${PROJECT_NAME}-shared PRIVATE
        BOOST_ALL_DYN_LINK
)

target_include_directories(${PROJECT_NAME}-shared PRIVATE
        $<BUILD_INTERFACE:
        ${PROJECT_SOURCE_DIR}/src/include
        ${PROJECT_SOURCE_DIR}/src/collision_detect/include
        ${PROJECT_SOURCE_DIR}/src/math/include
        ${PROJECT_SOURCE_DIR}/src/physics/include
        >

        $<INSTALL_INTERFACE:include/${PROJECT_NAME}-${PROJECT_VERSION}>
)

# 打包动态库
packageProject(
        # 要导出的 target
        NAME ${PROJECT_NAME}-shared
        # 版本
        VERSION ${PROJECT_VERSION}
        # 生成文件目录
        BINARY_DIR ${PROJECT_BINARY_DIR}
        # 头文件路径
        INCLUDE_DIR ${PROJECT_SOURCE_DIR}/src/include ${PROJECT_SOURCE_DIR}/src/collision_detect/include ${PROJECT_SOURCE_DIR}/src/math/include ${PROJECT_SOURCE_DIR}/src/physics/include
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

add_library(${PROJECT_NAME}-static STATIC
        ${lib_src}
)

target_include_directories(${PROJECT_NAME}-static PRIVATE
        $<BUILD_INTERFACE:
        ${PROJECT_SOURCE_DIR}/src/include
        ${PROJECT_SOURCE_DIR}/src/collision_detect/include
        ${PROJECT_SOURCE_DIR}/src/math/include
        ${PROJECT_SOURCE_DIR}/src/physics/include
        >

        $<INSTALL_INTERFACE:include/${PROJECT_NAME}-${PROJECT_VERSION}>

)

# 打包静态库
packageProject(
        # 要导出的 target
        NAME ${PROJECT_NAME}-static
        # 版本
        VERSION ${PROJECT_VERSION}
        # 生成文件目录
        BINARY_DIR ${PROJECT_BINARY_DIR}
        # 头文件路径
        INCLUDE_DIR ${PROJECT_SOURCE_DIR}/src/include ${PROJECT_SOURCE_DIR}/src/collision_detect/include ${PROJECT_SOURCE_DIR}/src/math/include ${PROJECT_SOURCE_DIR}/src/physics/include
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
