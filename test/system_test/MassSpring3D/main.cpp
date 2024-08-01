/**
 * @file ClothSimulation.cpp
 * @brief ClothSimulation main 实现
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

#include <chrono>
#include <cmath>
#include <iostream>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
 // #include <dtkScene.h>

#include "ClothSimulation.h"
#include "dtkStaticTriangleMesh.h"
#include "Shader.h"
#include "Renderer.h"

static auto last_clock = std::chrono::high_resolution_clock::now();

// The Width of the screen
const unsigned int WINDOW_WIDTH = 800;
// The height of the screen
const unsigned int WINDOW_HEIGHT = 600;
static ClothSimulation scene(WINDOW_WIDTH, WINDOW_HEIGHT, { 0, -9.8 });
// static ProgramInput* g_render_target; // vertex, index

static void checkGlErrors();

static void draw_text(int x, int y, const char* format, ...) {
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    int w = glutGet(GLUT_WINDOW_WIDTH);
    int h = glutGet(GLUT_WINDOW_HEIGHT);
    gluOrtho2D(0, w, h, 0);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glColor3f(0.9f, 0.9f, 0.9f);
    glRasterPos2i(x, y);

    char buffer[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, 256, format, args);
    va_end(args);
    for (int i = 0; i < len; ++i) {
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, buffer[i]);
    }

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0.0f, -8.0f, -25.0f);

    auto now = std::chrono::high_resolution_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_clock).count();
    last_clock = now;

    int h = glutGet(GLUT_WINDOW_HEIGHT);
    int w = glutGet(GLUT_WINDOW_WIDTH);

    draw_text(5, 20, "dtk @SoftBody simulation");

    draw_text(5, 40, "Push [1-1] to switch scene");
    draw_text(w - 150, h - 20, "refer: apollonia");

    if (scene.IsPause())
        draw_text(5, h - 20, "dt: %.2f ms PAUSED", dt * 1000);
    else
        draw_text(5, h - 20, "dt: %.2f ms", dt * 1000);

    scene.Update(std::min(dt, 0.08));
    scene.Render();

    GLenum err;
    while ((err = glGetError()) != GL_NO_ERROR) {
        std::cerr << "OpenGL error: " << err << std::endl;
    }

    glutSwapBuffers();
    checkGlErrors();
}

void reshape(int width, int height) {
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, width / (float)height, 0.1, 100.0);
}

void mouse(int button, int state, int x, int y) {}

void move_pos(const dtk::dtkDouble2& v) { scene.move(v); }

void keyboard(unsigned char key, int x, int y) {
    switch (key) {
    case '1':
        scene.SetVisible(!scene.IsVisible());
        break;
    case 'w':
        move_pos(dtk::dtkDouble2(0, 1));
        break;
    case 'a':
        move_pos(dtk::dtkDouble2(-1, 0));
        break;
    case 's':
        move_pos(dtk::dtkDouble2(0, -1));
        break;
    case 'd':
        move_pos(dtk::dtkDouble2(1, 0));
        break;
    case ' ':
        scene.SetPause(!scene.IsPause());
        break;
    case 27:
        exit(0);
        break;
    default:
        break;
    }
}

void motion(int x, int y) {}

void special(int key, int x, int y) {}

void idle() {
    display();
}

static void initGlutState(int argc, char** argv, const char* window_title = "", const unsigned int window_width = 800, const unsigned int window_height = 600) {
    glutInit(&argc, argv);
    glutInitWindowSize(window_width, window_height);
    glutInitWindowPosition(50, 50);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);  /// TODO
    glutCreateWindow(window_title);
    glutDisplayFunc(&display);
    glutReshapeFunc(&reshape);
    glutMouseFunc(&mouse);
    glutMotionFunc(&motion);
    glutSpecialFunc(&special);
    glutKeyboardFunc(&keyboard);
    glutIdleFunc(&idle);
}

static void initGlewState() {
    GLenum err = glewInit();
    if (!glewIsSupported("GL_VERSION_2_0")) {
        printf("OpenGL 2.0 not supported\n");
        exit(1);
    }
    if (err != GLEW_OK) {
        std::cerr << "Error initializing GLEW: " << glewGetErrorString(err) << std::endl;
        exit(1);
    }
}

static void checkGlErrors() {
    const GLenum errCode = glGetError();

    if (errCode != GL_NO_ERROR) {
        std::string error("GL Error: ");
        error += reinterpret_cast<const char*>(gluErrorString(errCode));
        std::cerr << error << std::endl;
        throw std::runtime_error(error);
    }
}

static void initGLState() {
    glClearColor(0.25f, 0.25f, 0.25f, 0);
    glClearDepth(1.);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glReadBuffer(GL_BACK);
    glEnable(GL_FRAMEBUFFER_SRGB);

    checkGlErrors();
}

int main(int argc, char* argv[]) {
    try {
        const char* window_title = "SimplePhysicsEngine-ST-MassSpring3D";
        initGlutState(argc, argv, window_title, WINDOW_WIDTH, WINDOW_HEIGHT);
        initGlewState();
        initGLState();

        scene.Init();
        checkGlErrors();

        glutMainLoop();

        scene.CleanUp();
        return 0;
    }
    catch (const std::runtime_error& e) {
        std::cout << "Exception caught: " << e.what() << std::endl;
        return -1;
    }
}