/**
 * @file ClothSimulation.cpp
 * @brief ClothSimulation 实现
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
#include "ClothSimulation.h"

ClothSimulation::ClothSimulation(const unsigned int& windowWidth, const unsigned int& windowHeight, const dtk::dtkDouble3& gravity) : Scene(windowWidth, windowHeight), _gravity(gravity) {
};

const ClothSimulation::ClothMesh ClothSimulation::GetClothMesh() const {
    return _cloth_mesh;
};

void ClothSimulation::move(const dtk::dtkDouble2& v) {

};

void ClothSimulation::CleanUp() {
    delete g_phongShader;
    delete g_pickShader;
    delete g_render_target;
};

void ClothSimulation::Init() {
    InitShader();
    InitCloth();
    InitScene();
};

void ClothSimulation::Update(float dt) {
    if (IsVisible() == false || IsPause() == true) return;

    _solver->solve(_iter_num);
    _solver->solve(_iter_num);

    // mCollisionDetectResponse->Update(dt, );
    _cloth_mesh->ComputeNormals();

    UpdateRenderTarget();
};

void ClothSimulation::Render() {
    if (!IsVisible()) return;
    Renderer renderer;
    renderer.setProgram(g_phongShader);
    renderer.setModelview(g_ModelViewMatrix);
    renderer.setProjection(g_ProjectionMatrix);
    g_phongShader->setAlbedo(g_albedo);
    g_phongShader->setAmbient(g_ambient);
    g_phongShader->setLight(g_light);
    renderer.setProgramInput(g_render_target);
    renderer.setElementCount(_cloth_mesh->GetNumberOfTriangles() * 3);
    renderer.draw();
};

void ClothSimulation::InitShader() {
    GLShader basic_vert(GL_VERTEX_SHADER);
    GLShader phong_frag(GL_FRAGMENT_SHADER);
    GLShader pick_frag(GL_FRAGMENT_SHADER);

    auto ibasic = std::ifstream("./test/system_test/MassSpring3D/shaders/basic.vshader");
    auto iphong = std::ifstream("./test/system_test/MassSpring3D/shaders/phong.fshader");
    auto ifrag = std::ifstream("./test/system_test/MassSpring3D/shaders/pick.fshader");

    basic_vert.compile(ibasic);
    phong_frag.compile(iphong);
    pick_frag.compile(ifrag);

    g_phongShader = new PhongShader;
    g_pickShader = new PickShader;
    g_phongShader->link(basic_vert, phong_frag);
    g_pickShader->link(basic_vert, pick_frag);
}

void ClothSimulation::InitCloth() {
    _cloth_mesh = dtkFactory::CreateClothMesh(SystemParam::w, SystemParam::n);

    ClothDrop();
    // ClothHang();

    g_render_target = new ProgramInput;
    UpdateRenderTarget();   // set position data
    const std::vector<dtk::dtkID3>& mEc = _cloth_mesh->GetECTable();
    unsigned int* indexBuffer = (unsigned int*)&mEc[0];
    unsigned int indexBufferSize = mEc.size() * 3;
    g_render_target->setIndexData(indexBuffer, indexBufferSize);
}

void ClothSimulation::InitScene() {
    g_ModelViewMatrix = glm::lookAt(
        glm::vec3(0.618, -0.786, 0.3f) * g_camera_distance,
        glm::vec3(0.0f, 0.0f, -1.0f),
        glm::vec3(0.0f, 0.0f, 1.0f)
    ) * glm::translate(glm::mat4(1), glm::vec3(0.0f, 0.0f, SystemParam::w / 4));
    g_ProjectionMatrix = glm::perspective(PI / 4.0f, g_windowWidth * 1.0f / g_windowHeight, 0.01f, 1000.0f);
};

void ClothSimulation::SetParameters() {

};

void ClothSimulation::ClothDrop() {
    _system = dtkFactory::CreateClothMassSpringSystem(_cloth_mesh);
    // _sphere_mesh = dtkFactory::CreateSphereMesh(dtk::dtkDouble3(0, 0, -1), 0.64, 20);

    _solver = dtkFactory::CreateClothMassSpringSolver(_system);

    mCollisionDetectResponse = dtk::dtkPhysMassSpringCollisionResponse::New();
    mCollisionDetectResponse->SetMassSpring(0, _system);
};

void ClothSimulation::ClothHang() {
    _system = dtkFactory::CreateClothMassSpringSystem(_cloth_mesh);

    _solver = dtkFactory::CreateClothMassSpringSolver(_system);

    mCollisionDetectResponse = dtk::dtkPhysMassSpringCollisionResponse::New();
    mCollisionDetectResponse->SetMassSpring(0, _system);
};

void ClothSimulation::UpdateRenderTarget() {
    dtk::dtkPoints::Ptr mPts = _cloth_mesh->GetPoints();

    float* vertexBuffer = _solver->getVertexBuffer();
    unsigned int vertexBufferSize = mPts->GetNumberOfPoints() * 3;

    // std::cout << "vertexBuffer:" << std::endl;
    // for (int i = 0;i < vertexBufferSize;i++) {
    //     std::cout << vertexBuffer[i] << " ";
    // }
    // std::cout << std::endl;

    g_render_target->setPositionData(vertexBuffer, vertexBufferSize);
};

dtk::dtkStaticTriangleMesh::Ptr dtkFactory::CreateClothMesh(float w, int n) {
    dtk::dtkStaticTriangleMesh::Ptr result = dtk::dtkStaticTriangleMesh::New();

    unsigned int idx = 0; // vertex index
    const float d = w / (n - 1); // step distance
    const float ud = 1.0f / (n - 1); // unit step distance
    const glm::vec3 o = glm::vec3(-w / 2.0f, w / 2.0f, 0.0f); // origin
    const glm::vec3 ux = glm::vec3(1.0f, 0.0f, 0.0f); // unit x direction
    const glm::vec3 uy = glm::vec3(0.0f, -1.0f, 0.0f); // unit y direction

    dtk::dtkPointsVector::Ptr vertices = dtk::dtkPointsVector::New();
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            glm::vec3 p = o + d * j * ux + d * i * uy; // vertex position
            dtk::GK::Point3 p3(p.x, p.y, p.z);
            dtk::dtkID pid = j + i * n;
            vertices->InsertPoint(pid, p3); // add vertex
        }
    }
    result->SetPoints(vertices);

    for (int i = 0;i < n;i++) {
        for (int j = 0;j < n;j++) {
            //add connectivity
            if (i > 0 && j < n - 1) {
                result->InsertTriangleNotRepeat(
                    j + i * n,
                    j + 1 + (i - 1) * n,
                    j + (i - 1) * n
                );
            }

            if (j > 0 && i > 0) {
                result->InsertTriangleNotRepeat(
                    j + i * n,
                    j + (i - 1) * n,
                    j - 1 + i * n
                );
            }
        }
    }

    result->ComputeNormals();
    return result;
}

dtk::dtkStaticTriangleMesh::Ptr dtkFactory::CreateSphereMesh(dtk::dtkDouble3 center, float radius, int n) {
    dtk::dtkStaticTriangleMesh::Ptr result = dtk::dtkStaticTriangleMesh::New();
    dtk::dtkPointsVector::Ptr vertices = dtk::dtkPointsVector::New();

    // Generate vertices
    for (int i = 0; i <= n; ++i) {
        float theta = i * glm::pi<float>() / n;
        for (int j = 0; j <= n; ++j) {
            float phi = j * 2 * glm::pi<float>() / n;
            float x = center.x + radius * sin(theta) * cos(phi);
            float y = center.y + radius * sin(theta) * sin(phi);
            float z = center.z + radius * cos(theta);
            dtk::GK::Point3 p3(x, y, z);
            dtk::dtkID pid = j + i * (n + 1);
            vertices->InsertPoint(pid, p3);
        }
    }
    result->SetPoints(vertices);

    // Generate triangles
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            dtk::dtkID p1 = j + i * (n + 1);
            dtk::dtkID p2 = j + (i + 1) * (n + 1);
            dtk::dtkID p3 = (j + 1) + i * (n + 1);
            dtk::dtkID p4 = (j + 1) + (i + 1) * (n + 1);

            if (i != 0) {
                result->InsertTriangleNotRepeat(p1, p2, p3);
            }
            if (i != (n - 1)) {
                result->InsertTriangleNotRepeat(p3, p2, p4);
            }
        }
    }

    return result;
}

dtk::dtkPhysMassSpring::Ptr dtkFactory::CreateClothMassSpringSystem(const dtk::dtkStaticTriangleMesh::Ptr& mesh) {
    dtk::dtkDouble3 gravity(0, 0, -SystemParam::g);
    dtk::dtkPhysMassSpring::Ptr system = dtk::dtkPhysMassSpring::New(SystemParam::m, SystemParam::k, SystemParam::b, SystemParam::a, SystemParam::r, SystemParam::h, gravity);
    // system->SetTriangleMesh(mesh);

    // n must be odd
    assert(SystemParam::n % 2 == 1);

    // compute n_points and n_springs
    unsigned int n_points = SystemParam::n * SystemParam::n;
    unsigned int n_springs = (SystemParam::n - 1) * (5 * SystemParam::n - 2);
    system->SetPoints(mesh->GetPoints());
    for (unsigned int id = 0; id < n_points; id++) {
        system->AddMassPoint(id, SystemParam::m, dtk::dtkT3<double>(0, 0, 0), SystemParam::a, SystemParam::c, gravity);
    }

    unsigned int n = SystemParam::n;
    double rest_length_factor = 1.05;
    for (unsigned int i = 0; i < SystemParam::n; i++) {
        for (unsigned int j = 0; j < SystemParam::n; j++) {
            // bottom right corner
            if (i == n - 1 && j == n - 1) {
                continue;
            }

            if (i == n - 1) {
                // structural spring
                system->AddSpring(n * i + j, n * i + j + 1, SystemParam::k, SystemParam::a, rest_length_factor);

                // bending spring
                if (j % 2 == 0) {
                    system->AddSpring(n * i + j, n * i + j + 2, SystemParam::k, SystemParam::a, rest_length_factor);
                }
                continue;
            }

            // right edge
            if (j == n - 1) {
                // structural spring
                system->AddSpring(n * i + j, n * (i + 1) + j, SystemParam::k, SystemParam::a, rest_length_factor);

                // bending spring
                if (i % 2 == 0) {
                    system->AddSpring(n * i + j, n * (i + 2) + j, SystemParam::k, SystemParam::a, rest_length_factor);
                }
                continue;
            }

            // structural springs
            system->AddSpring(n * i + j, n * i + j + 1, SystemParam::k, SystemParam::a, rest_length_factor);

            system->AddSpring(n * i + j, n * (i + 1) + j, SystemParam::k, SystemParam::a, rest_length_factor);

            // shearing springs
            system->AddSpring(n * i + j, n * (i + 1) + j + 1, SystemParam::k, SystemParam::a, rest_length_factor);

            system->AddSpring(n * (i + 1) + j, n * i + j + 1, SystemParam::k, SystemParam::a, rest_length_factor);

            // bending springs
            if (j % 2 == 0) {
                system->AddSpring(n * i + j, n * i + j + 2, SystemParam::k, SystemParam::a, rest_length_factor);
            }
            if (i % 2 == 0) {
                system->AddSpring(n * i + j, n * (i + 2) + j, SystemParam::k, SystemParam::a, rest_length_factor);
            }
        }
    }

    // TODO: setting gravity as an external force

    return system;
}

dtk::dtkPhysMassSpringSolver::Ptr dtkFactory::CreateClothMassSpringSolver(const dtk::dtkPhysMassSpring::Ptr& system) {
    dtk::dtkPhysMassSpringSolver::Ptr solver = dtk::dtkPhysMassSpringSolver::New(system);
    return solver;
}

