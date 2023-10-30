
# This file is a part of Simple-XX/SimplePhysicsEngine
# (https://github.com/Simple-XX/SimplePhysicsEngine).
#
# CMakeLists.txt for Simple-XX/SimplePhysicsEngine.

project(dtk)

option(GROUP_BY_EXPLORER ON)

# aux_source_directory(. dir_source)
file(GLOB_RECURSE dir_source *.cpp)
file(GLOB_RECURSE dir_headers *.h)

set(all_files ${dir_source} ${dir_headers})

foreach(fileItem ${all_files})       
    # Get the directory of the source file
    get_filename_component(PARENT_DIR "${fileItem}" DIRECTORY)
    # Remove common directory prefix to make the group
    string(REPLACE "${CMAKE_CURRENT_SOURCE_DIR}" "" GROUP "${PARENT_DIR}")
    # Make sure we are using windows slashes
    string(REPLACE "/" "\\" GROUP "${GROUP}")
    # Group into "Source Files" and "Header Files"
    set(GROUP "${GROUP}")
    source_group("${GROUP}" FILES "${fileItem}")
endforeach()

add_library(dtk STATIC 
    dtk/dtk.cpp
    dtk/dtk.h
    dtk/dtkArray.h
    dtk/dtkAssert.cpp
    dtk/dtkAssert.h
    dtk/dtkCollisionDetectBasic.cpp
    dtk/dtkCollisionDetectBasic.h
    dtk/dtkCollisionDetectHierarchy.cpp
    dtk/dtkCollisionDetectHierarchy.h
    dtk/dtkCollisionDetectHierarchyKDOPS.cpp
    dtk/dtkCollisionDetectHierarchyKDOPS.h
    dtk/dtkCollisionDetectNode.cpp
    dtk/dtkCollisionDetectNode.h
    dtk/dtkCollisionDetectNodeKDOPS.cpp
    dtk/dtkCollisionDetectNodeKDOPS.h
    dtk/dtkCollisionDetectPrimitive.cpp
    dtk/dtkCollisionDetectPrimitive.h
    dtk/dtkCollisionDetectStage.cpp
    # dtk/dtkCollisionDetectStage.cpp.bak
    dtk/dtkCollisionDetectStage.h
    dtk/dtkConfig.h
    dtk/dtkError.h
    dtk/dtkErrorManager.cpp
    dtk/dtkErrorManager.h
    dtk/dtkExports.h
    dtk/dtkGraphics.h
    dtk/dtkGraphicsKernel.cpp
    dtk/dtkGraphicsKernel.h
    # dtk/dtkGraphicsKernel.h.bak
    dtk/dtkGraphicsTools.h
    dtk/dtkIDTypes.h
    dtk/dtkImports.h
    dtk/dtkInit.h
    dtk/dtkIntersectTest.cpp
    dtk/dtkIntersectTest.h
    dtk/dtkIO.h
    dtk/dtkMatrixAlgorithm.h
    dtk/dtkPhysCore.cpp
    # dtk/dtkPhysCore.cpp.bak
    dtk/dtkPhysCore.h
    dtk/dtkPhysKnotPlanner.cpp
    dtk/dtkPhysKnotPlanner.h
    dtk/dtkPhysMassPoint.cpp
    dtk/dtkPhysMassPoint.h
    dtk/dtkPhysMassSpring.cpp
    dtk/dtkPhysMassSpring.h
    # dtk/dtkPhysMassSpring.h.bak
    dtk/dtkPhysMassSpringCollisionResponse.cpp
    dtk/dtkPhysMassSpringCollisionResponse.h
    dtk/dtkPhysMassSpringThread.cpp
    dtk/dtkPhysMassSpringThread.h
    dtk/dtkPhysMassSpringThreadCollisionResponse.cpp
    dtk/dtkPhysMassSpringThreadCollisionResponse.h
    dtk/dtkPhysParticle.cpp
    dtk/dtkPhysParticle.h
    dtk/dtkPhysParticleSystem.cpp
    dtk/dtkPhysParticleSystem.h
    dtk/dtkPhysSpring.cpp
    dtk/dtkPhysSpring.h
    dtk/dtkPhysTetraMassSpring.cpp
    dtk/dtkPhysTetraMassSpring.h
    dtk/dtkPoints.h
    dtk/dtkPointsReader.cpp
    dtk/dtkPointsReader.h
    dtk/dtkPointsVector.h
    dtk/dtkPointsWriter.cpp
    dtk/dtkPointsWriter.h
    dtk/dtkProperty.h
    dtk/dtkRandom.h
    dtk/dtkScene.cpp
    dtk/dtkScene.h
    dtk/dtkSign.cpp
    dtk/dtkSign.h
    dtk/dtkStaticMeshEliminator.cpp
    dtk/dtkStaticMeshEliminator.h
    dtk/dtkStaticTetraMesh.cpp
    dtk/dtkStaticTetraMesh.h
    dtk/dtkStaticTetraMeshReader.cpp
    dtk/dtkStaticTetraMeshReader.h
    dtk/dtkStaticTetraMeshWriter.cpp
    dtk/dtkStaticTetraMeshWriter.h
    dtk/dtkStaticTriangleMesh.cpp
    dtk/dtkStaticTriangleMesh.h
    dtk/dtkStaticTriangleMeshReader.cpp
    dtk/dtkStaticTriangleMeshReader.h
    dtk/dtkStaticTriangleMeshWriter.cpp
    dtk/dtkStaticTriangleMeshWriter.h
    dtk/dtkTime.h
    dtk/dtkUtility.h

    math/dtkMatrix.h
    math/dtkMatrixOp.cpp
    math/dtkMatrixOP.h
    math/dtkTx.h
    math/dtkTxIO.h
    math/dtkTxOP.h

    physics/dtkJoint.cpp
    physics/dtkJoint.h
    physics/dtkRigidBody.cpp
    physics/dtkRigidBody.h
    
    ${all_files}
)

target_include_directories(dtk PRIVATE
    dtk
    math
    physics
) 

target_link_libraries(dtk PRIVATE
    Boost 
    CGAL
    GLUT
    glfw
    glm
)

#[[install(
    TARGETS dtk
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

# install
install(TARGETS dtk DESTINATION ${DTK_INSTALL_LIBDIR})
install(FILES ${dir_headers} DESTINATION ${DTK_INSTALL_INCLUDEDIR})]]

# # 打包
# # @todo 导出静态库与动态库
# packageProject(
#         # 要导出的 target
#         NAME dtk
#         # 版本
#         VERSION ${PROJECT_VERSION}
#         # 生成文件目录
#         BINARY_DIR ${PROJECT_BINARY_DIR}
#         # 头文件路径
#         INCLUDE_DIR ${PROJECT_SOURCE_DIR}/src/include
#         # 与 target 的 INSTALL_INTERFACE 一致
#         INCLUDE_DESTINATION include/${PROJECT_NAME}-${PROJECT_VERSION}
#         # 头文件过滤
#         INCLUDE_HEADER_PATTERN "*.h"
#         # namespace 名称，不能包含 '::'
#         NAMESPACE ${PROJECT_NAMESPACE}
#         # (optional) option to disable the versioning of install destinations
#         DISABLE_VERSION_SUFFIX YES
#         ARCH_INDEPENDENT YES
#         # 生成 CPack 变量
#         CPACK YES
# )
