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

#include <GL/freeglut.h>
#include <dtkScene.h>

#include "ClothSimulation.h"

static auto last_clock = std::chrono::high_resolution_clock::now();

// The Width of the screen
const unsigned int SCREEN_WIDTH = 800;
// The height of the screen
const unsigned int SCREEN_HEIGHT = 600;
static ClothSimulation world({ 0, -9.8 });

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

static void test_cloth_simulation() {
    std::cout << "test" << std::endl;
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0.0f, -8.0f, -25.0f);

    auto now = std::chrono::high_resolution_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(
        now - last_clock)
        .count();
    last_clock = now;

    int h = glutGet(GLUT_WINDOW_HEIGHT);
    int w = glutGet(GLUT_WINDOW_WIDTH);

    draw_text(5, 20, "dtk @SoftBody simulation");

    draw_text(5, 40, "Push [1-1] to switch scene");
    draw_text(w - 150, h - 20, "refer: apollonia");

    if (world.is_pause())
        draw_text(5, h - 20, "dt: %.2f ms PAUSED", dt * 1000);
    else
        draw_text(5, h - 20, "dt: %.2f ms", dt * 1000);

    world.step(std::min(dt, 0.01));

    glutSwapBuffers();
}

void reshape(int width, int height) {
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, width / (float)height, 0.1, 100.0);
}

void mouse(int button, int state, int x, int y) {}

void move_pos(const dtk::dtkDouble2& v) { world.move(v); }

void keyboard(unsigned char key, int x, int y) {
    switch (key) {
    case '1':
        world.clear();
        test_cloth_simulation();
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
        world.set_pause(!world.is_pause());
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

void idle() { display(); }

int main(int argc, char* argv[]) {
    glutInit(&argc, argv);
    glutInitWindowSize(SCREEN_WIDTH, SCREEN_HEIGHT);
    glutInitWindowPosition(50, 50);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
    glutCreateWindow("SimplePhysicsEngine-ST-MassSpring3D");
    // world.Init();
    glutDisplayFunc(&display);
    glutReshapeFunc(&reshape);
    glutMouseFunc(&mouse);
    glutMotionFunc(&motion);
    glutSpecialFunc(&special);
    glutKeyboardFunc(&keyboard);
    glutIdleFunc(&idle);
    /// @todo not found
    // glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
    glutMainLoop();
    return 0;
}