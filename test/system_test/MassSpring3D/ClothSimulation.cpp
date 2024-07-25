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
    // if (IsVisible() == false || IsPause() == true) return;

    UpdateRenderTarget();
};

void ClothSimulation::Render() {
    // glMatrixMode(GL_PROJECTION);
    // glLoadMatrixf(glm::value_ptr(g_ProjectionMatrix));
    // glMatrixMode(GL_MODELVIEW);
    // glLoadMatrixf(glm::value_ptr(g_ModelViewMatrix));
    // glColor3f(0.8f, 0.8f, 0.0f);
    // const std::vector<dtk::dtkID3>& points = _cloth_mesh->GetECTable();
    // for (int i = 0; i < points.size(); i++) {
    //     dtk::dtkID3 p = points[i];
    //     glBegin(GL_TRIANGLES);
    //     glVertex3f(_cloth_mesh->GetPoint(p[0])[0], _cloth_mesh->GetPoint(p[0])[1], _cloth_mesh->GetPoint(p[0])[2]);
    //     glVertex3f(_cloth_mesh->GetPoint(p[1])[0], _cloth_mesh->GetPoint(p[1])[1], _cloth_mesh->GetPoint(p[1])[2]);
    //     glVertex3f(_cloth_mesh->GetPoint(p[2])[0], _cloth_mesh->GetPoint(p[2])[1], _cloth_mesh->GetPoint(p[2])[2]);
    //     glEnd();
    // }

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

    g_render_target = new ProgramInput;

    UpdateRenderTarget();

    ClothDrop();
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
    _system = dtkFactory::CreateClothMassSpring(_cloth_mesh, _gravity);
};

void ClothSimulation::UpdateRenderTarget() {
    dtk::dtkPoints::Ptr mPts = _cloth_mesh->GetPoints();
    const std::vector<dtk::dtkID3>& mEc = _cloth_mesh->GetECTable();

    float* vertexBuffer = new float[mPts->GetNumberOfPoints() * 3];
    for (int i = 0; i < mPts->GetNumberOfPoints(); i++) {
        dtk::GK::Point3 p = mPts->GetPoint(i);
        vertexBuffer[i * 3] = p[0];
        vertexBuffer[i * 3 + 1] = p[1];
        vertexBuffer[i * 3 + 2] = p[2];
    }
    unsigned int vertexBufferSize = mPts->GetNumberOfPoints() * 3;
    unsigned int* indexBuffer = (unsigned int*)&mEc[0];
    unsigned int indexBufferSize = mEc.size() * 3;

    g_render_target->setPositionData(vertexBuffer, vertexBufferSize);
    g_render_target->setIndexData(indexBuffer, indexBufferSize);
};

dtk::dtkStaticTriangleMesh::Ptr dtkFactory::CreateClothMesh(float w, int n) {
    dtk::dtkStaticTriangleMesh::Ptr result = dtk::dtkStaticTriangleMesh::New();

    unsigned int idx = 0; // vertex index
    const float d = w / (n - 1); // step distance
    const float ud = 1.0f / (n - 1); // unit step distance
    const glm::vec3 o = glm::vec3(-w / 2.0f, w / 2.0f, 0.0f); // origin
    const glm::vec3 ux = glm::vec3(1.0f, 0.0f, 0.0f); // unit x direction
    const glm::vec3 uy = glm::vec3(0.0f, -1.0f, 0.0f); // unit y direction
    std::vector<dtk::GK::Point3> handle_table(n * n); // table storing vertex handles for easy grid connectivity establishment

    dtk::dtkPointsVector::Ptr vertices = dtk::dtkPointsVector::New();
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            glm::vec3 p = o + d * j * ux + d * i * uy; // vertex position
            dtk::GK::Point3 p3(p.x, p.y, p.z);
            dtk::dtkID pid = j + i * n;
            vertices->InsertPoint(pid, p3); // add vertex
            // handle_table[pid] = vertices->GetPoint(pid); // store vertex handle
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
    return result;
}

dtk::dtkPhysMassSpring::Ptr dtkFactory::CreateClothMassSpring(const dtk::dtkStaticTriangleMesh::Ptr& mesh, const dtk::dtkDouble3& gravity) {
    dtk::dtkPhysMassSpring::Ptr system = dtk::dtkPhysMassSpring::New(SystemParam::m, SystemParam::k, SystemParam::b, SystemParam::a, SystemParam::r, gravity);
    system->SetTriangleMesh(mesh);

    // n must be odd
    assert(SystemParam::n % 2 == 1);

    // compute n_points and n_springs
    unsigned int n_points = SystemParam::n * SystemParam::n;
    unsigned int n_springs = (SystemParam::n - 1) * (5 * SystemParam::n - 2);

    for (unsigned int id = 0; id < n_points; id++) {
        system->AddMassPoint(id, SystemParam::m, dtk::dtkT3<double>(0, 0, 0), SystemParam::a, SystemParam::c, gravity);
    }

    unsigned int n = SystemParam::n;
    unsigned int k = 0; // spring counter
    for (unsigned int i = 0; i < SystemParam::n; i++) {
        for (unsigned int j = 0; j < SystemParam::n; j++) {
            // bottom right corner
            if (i == n - 1 && j == n - 1) {
                continue;
            }

            if (i == n - 1) {
                // structural spring
                system->AddSpring(n * i + j, n * i + j + 1, SystemParam::k, SystemParam::a);

                // bending spring
                if (j % 2 == 0) {
                    system->AddSpring(n * i + j, n * i + j + 2, SystemParam::k, SystemParam::a);
                }
                continue;
            }

            // right edge
            if (j == n - 1) {
                // structural spring
                system->AddSpring(n * i + j, n * (i + 1) + j, SystemParam::k, SystemParam::a);

                // bending spring
                if (i % 2 == 0) {
                    system->AddSpring(n * i + j, n * (i + 2) + j, SystemParam::k, SystemParam::a);
                }
                continue;
            }

            // structural springs
            system->AddSpring(n * i + j, n * i + j + 1, SystemParam::k, SystemParam::a);

            system->AddSpring(n * i + j, n * (i + 1) + j, SystemParam::k, SystemParam::a);

            // shearing springs
            system->AddSpring(n * i + j, n * (i + 1) + j + 1, SystemParam::k, SystemParam::a);

            system->AddSpring(n * (i + 1) + j, n * i + j + 1, SystemParam::k, SystemParam::a);

            // bending springs
            if (j % 2 == 0) {
                system->AddSpring(n * i + j, n * i + j + 2, SystemParam::k, SystemParam::a);
            }
            if (i % 2 == 0) {
                system->AddSpring(n * i + j, n * (i + 2) + j, SystemParam::k, SystemParam::a);
            }
        }
    }

    return system;
}