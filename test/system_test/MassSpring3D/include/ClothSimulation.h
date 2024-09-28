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

class SystemParam {
public:
    SystemParam(int n = 33, float w = 2.0f, float h = 0.008f, float k = 1.0f,
                float a = 0.993f, float b = 5880.0f, float c = 2.5f)
        : n(n), w(w), h(h), r(w / (n - 1) * 1.05f), k(k),
        m(0.05f / (n * n)), a(a), b(b), c(c), g(9.8f * m) {}

    const int n; // must be odd, n * n = n_vertices | 33
    const float w; // width | 2.0f
    const float h; // time step, smaller for better results | 0.008f = 0.016f/2
    const float r; // spring rest length
    const float k; // spring stiffness | 1.0f
    const float m; // point mass | 0.25f
    const float a; // point damping, close to 1.0 | 0.993f
    const float b; // damping | 5880.0f
    const float c; // point resistance | 2.5f
    const float g; // gravitational force | 9.8f
};

class ClothSimulation : public Scene {
    typedef Eigen::VectorXf VectorXf;
public:
    ClothSimulation(const unsigned int& windowWidth, const unsigned int& windowHeight, const dtk::dtkDouble3& gravity, const unsigned int& edge_num);
    ~ClothSimulation() = default;

    using ClothMesh = dtk::dtkStaticTriangleMesh::Ptr;
    using SphereMesh = dtk::dtkStaticTriangleMesh::Ptr;
    using ClothMassSpring = dtk::dtkPhysMassSpring::Ptr;
    using ClothMassSpringSolver = dtk::dtkPhysMassSpringSolver::Ptr;

    const ClothMesh GetClothMesh() const;

    void move(const dtk::dtkDouble3& v);
    void rotate_view(const float& deltaX, const float& deltaY);

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

    void UpdateClothMesh();

    // Shader
    PhongShader* g_phongShader; // linked phong shader
    PickShader* g_pickShader; // linked pick shader

    // Camera
    dtk::dtkMatrix44 g_ModelViewMatrix;
    dtk::dtkMatrix44 g_ProjectionMatrix;
    glm::vec3 g_camera_position;
    glm::vec3 g_camera_target;
    glm::vec3 g_camera_up;
    const float g_camera_distance = 4.2f;
    const float PI = glm::pi<float>();

    ProgramInput* g_render_target; // vertex, index

    // @todo: change to dtk vector
    const glm::vec3 g_albedo = glm::vec3(0.0f, 0.2f, 0.9f);
    const glm::vec3 g_ambient = glm::vec3(0.01f, 0.01f, 0.01f);
    const glm::vec3 g_light = glm::vec3(1.0f, 1.0f, -1.0f);

    // 重力
    dtk::dtkDouble3 _gravity;

    SystemParam _param;
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
    static dtk::dtkPhysMassSpring::Ptr CreateClothMassSpringSystem(const dtk::dtkStaticTriangleMesh::Ptr& mesh, const SystemParam& _param);
    static dtk::dtkPhysMassSpringSolver::Ptr CreateClothMassSpringSolver(const dtk::dtkPhysMassSpring::Ptr& system);
    static dtk::dtkStaticTriangleMesh::Ptr CreateSphereMesh(dtk::dtkDouble3 center, float radius, int n);
};

#endif /* SIMPLEPHYSICSENGINE_CLOTHSIMULATION_H */
