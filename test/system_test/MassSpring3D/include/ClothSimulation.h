/**
 * @file ClothSimulation.h
 * @brief ClothSimulation 头文件
 * @author Chance
 * @version 1.0
 * @date 2024-07-12
 * @copyright MIT LICENSE
 * https://github.com/Simple-XX/SimplePhysicsEngine
 * @par change log:
 * <table>
 * <tr><th>Date<th>Author<th>Description
 * <tr><td>2024-07-12<td>Chance<td>创建文件
 * </table>
 */
#ifndef SIMPLEPHYSICSENGINE_CLOTHSIMULATION_H
#define SIMPLEPHYSICSENGINE_CLOTHSIMULATION_H

#include <iostream>
#include <memory>
#include <map>
#include <unordered_map>
#include <vector>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <dtkJoint.h>

#include "dtkStaticTriangleMesh.h"
#include "dtkPhysMassSpring.h"
#include "dtkMatrix.h"

#include "Scene.h"
#include "Shader.h"
#include "Renderer.h"
#include "dtkCollisionDetectHierarchyKDOPS.h"
#include "dtkPhysCore.h"
#include "dtkPhysMassSpringSolver.h"
#include "dtkPhysMassSpringCollisionResponse.h"

namespace SystemParam {
    static const int n = 33; // must be odd, n * n = n_vertices | 61
    static const float w = 2.0f; // width | 2.0f
    static const float h = 0.008f; // time step, smaller for better results | 0.008f = 0.016f/2
    static const float r = w / (n - 1) * 1.05f; // spring rest legnth
    static const float k = 1.0f; // spring stiffness | 1.0f;
    static const float m = 0.25f / (n * n); // point mass | 0.25f
    static const float a = 0.993f; // point damping, close to 1.0 | 0.993f
    static const float b = 5880.0f; // damping | 5880.0f
    static const float c = 2.5f; // point resistence | 2.5f
    static const float g = 9.8f * m; // gravitational force | 9.8f
}

class ClothSimulation : public Scene {
public:
    ClothSimulation(const unsigned int& windowWidth, const unsigned int& windowHeight, const dtk::dtkDouble3& gravity);
    ~ClothSimulation() = default;

    using ClothMesh = dtk::dtkStaticTriangleMesh::Ptr;
    using SphereMesh = dtk::dtkStaticTriangleMesh::Ptr;
    using ClothMassSpring = dtk::dtkPhysMassSpring::Ptr;
    using ClothMassSpringSolver = dtk::dtkPhysMassSpringSolver::Ptr;

    const ClothMesh GetClothMesh() const;

    void move(const dtk::dtkDouble2& v);

    void CleanUp();

    void Init();
    void Update(float dt);
    void Render();
    void UpdateRenderTarget();

private:
    void InitShader();
    void InitCloth();
    void InitScene();
    void SetParameters();
    void ClothDrop();
    void ClothHang();

    // Shader
    PhongShader* g_phongShader; // linked phong shader
    PickShader* g_pickShader; // linked pick shader

    // Camera
    dtk::dtkMatrix44 g_ModelViewMatrix;
    dtk::dtkMatrix44 g_ProjectionMatrix;
    const float g_camera_distance = 4.2f;
    const float PI = glm::pi<float>();

    ProgramInput* g_render_target; // vertex, index

    // @todo: change to dtk vector
    const glm::vec3 g_albedo = glm::vec3(0.0f, 0.3f, 0.7f);
    const glm::vec3 g_ambient = glm::vec3(0.01f, 0.01f, 0.01f);
    const glm::vec3 g_light = glm::vec3(1.0f, 1.0f, -1.0f);

    // 重力
    dtk::dtkDouble3 _gravity;

    ClothMesh _cloth_mesh;
    SphereMesh _sphere_mesh;
    ClothMassSpring _system;
    ClothMassSpringSolver _solver;

    dtk::dtkPhysMassSpringCollisionResponse::Ptr mCollisionDetectResponse;

    const int _iter_num = 5;
};

class dtkFactory {
public:
    static dtk::dtkStaticTriangleMesh::Ptr CreateClothMesh(float w, int n);
    static dtk::dtkPhysMassSpring::Ptr CreateClothMassSpringSystem(const dtk::dtkStaticTriangleMesh::Ptr& mesh);
    static dtk::dtkPhysMassSpringSolver::Ptr CreateClothMassSpringSolver(const dtk::dtkPhysMassSpring::Ptr& system);
    static dtk::dtkStaticTriangleMesh::Ptr CreateSphereMesh(dtk::dtkDouble3 center, float radius, int n);
};

#endif /* SIMPLEPHYSICSENGINE_CLOTHSIMULATION_H */
