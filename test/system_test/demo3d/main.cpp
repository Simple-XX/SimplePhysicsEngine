﻿
/**
 * @file main.cpp
 * @brief main 实现
 * @author TOMsworkspace (2683322180@qq.com)
 * @version 1.0
 * @date 2021-09-13
 * @copyright MIT LICENSE
 * https://github.com/Simple-XX/SimplePhysicsEngine
 * @par change log:
 * <table>
 * <tr><th>Date<th>Author<th>Description
 * <tr><td>2021-09-13<td>TOMsworkspace<td>创建文件
 * <tr><td>2023-10-31<td>Zone.N<td>迁移到 doxygen
 * </table>
 */

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include <GL/freeglut.h>

#include "FemSimulation.h"

static auto last_clock = std::chrono::high_resolution_clock::now();

// The Width of the screen
const unsigned int SCREEN_WIDTH = 800;
// The height of the screen
const unsigned int SCREEN_HEIGHT = 600;

FemSimulation Breakout(SCREEN_WIDTH, SCREEN_HEIGHT);

#ifdef USE_VTK

#include "vtkActor.h"
#include "vtkCamera.h"
#include "vtkCellArray.h"
#include "vtkFloatArray.h"
#include "vtkInteractorStyleImage.h"
#include "vtkPointData.h"
#include "vtkPoints.h"
#include "vtkPolyData.h"
#include "vtkPolyDataMapper.h"
#include "vtkProperty.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include <vtkAnimationScene.h>
#include <vtkDataSetMapper.h>
#include <vtkLineSource.h>
#include <vtkSphereSource.h>

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

void vtk_vis() {

  Breakout.Init();
  auto pts = Breakout.mesh_index_list;
  vtkCellArray *polys = vtkCellArray::New();
  int face_cnt = Breakout.mesh_index_list.size();
  for (int i = 0; i < face_cnt; i++) {
    vtkIdType face_idx[4] = {pts[i][0], pts[i][1], pts[i][2], pts[i][3]};
    polys->InsertNextCell(4, face_idx);
  }
  vtkRenderWindow *renWin = vtkRenderWindow::New();
  while (1) {
    auto x = Breakout.points;
    vtkPolyData *cube = vtkPolyData::New();
    vtkPoints *points = vtkPoints::New();

    int point_cnt = Breakout.points.size();
    for (int i = 0; i < point_cnt; i++)
      points->InsertPoint(i, x[i].x(), x[i].y(), x[i].z());

    cube->SetPoints(points);
    cube->SetPolys(polys);

    vtkSphereSource *sphereSource = vtkSphereSource::New();
    sphereSource->SetCenter(Breakout.sphere.x, Breakout.sphere.y,
                            Breakout.sphere.z);      // 设置中心
    sphereSource->SetRadius(Breakout.sphere.radius); // 设置半径

    vtkPolyDataMapper *cubeMapper = vtkPolyDataMapper::New(),
                      *sphereMapper = vtkPolyDataMapper::New();
    cubeMapper->SetInputData(cube);
    sphereMapper->SetInputConnection(sphereSource->GetOutputPort());
    vtkActor *cubeActor = vtkActor::New(), *sphereActor = vtkActor::New();
    cubeActor->SetMapper(cubeMapper);
    sphereActor->SetMapper(sphereMapper);
    vtkCamera *camera = vtkCamera::New();
    camera->SetPosition(1, 1, 1);
    camera->SetFocalPoint(0, 0, 0);
    vtkRenderer *renderer = vtkRenderer::New();
    renWin->AddRenderer(renderer);
    renderer->AddActor(cubeActor);
    renderer->AddActor(sphereActor);
    renderer->SetActiveCamera(camera);
    renderer->ResetCamera();
    renderer->SetBackground(1, 1, 1);
    renWin->SetSize(SCREEN_WIDTH, SCREEN_HEIGHT);

    renWin->Render();

    auto now = std::chrono::high_resolution_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(
                  now - last_clock)
                  .count();
    last_clock = now;
    Breakout.Update(dt);
  }
  renWin->Delete();
}

#endif // VTK_OPT

static void draw_text(int x, int y, const char *format, ...) {
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  int w = glutGet(GLUT_WINDOW_WIDTH);
  int h = glutGet(GLUT_WINDOW_HEIGHT);
  gluOrtho2D(0, w, h, 0);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  glColor3f(255.0 / 255.0, 240.0 / 255.0, 97.0 / 255.0);
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
  glClearColor(3.0 / 255.0, 6.0 / 255.0, 13.0 / 255.0, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslatef(-0.5f, -0.55f, -1.0f);

  auto now = std::chrono::high_resolution_clock::now();
  auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(
                now - last_clock)
                .count();
  last_clock = now;

  int h = glutGet(GLUT_WINDOW_HEIGHT);
  int w = glutGet(GLUT_WINDOW_WIDTH);

  Breakout.Update(dt);
  Breakout.Render();
  glDisable(GL_DEPTH_TEST);
  draw_text(5, 15, "dtk @Deformation FEM simulation");

  draw_text(5, 30, "Method: Semi-implict Euler");
  // draw_text(5, 45, "Time step: %.6f", 1.0 / 32);
  draw_text(5, 45, "Poisson's ratio: %.2f", 0.3f);
  draw_text(5, 60, "Young's modulus: %.2f", 1000.0f);
  draw_text(5, 75, "Energy : %.2f", Breakout.getEnergy());

  // draw_text(5, 40, "Push [1-5] to switch scene");
  // draw_text(w - 150, h - 20, "refer: apollonia");

  if (Breakout.State == dtk::SCENE_PAUSE)
    draw_text(5, h - 20, "dt: %.2f ms (%.2F FPS) PAUSED", dt * 1000, 1.0 / dt);
  else
    draw_text(5, h - 20, "dt: %.2f ms (%.2F FPS)", dt * 1000, 1.0 / dt);

  glutSwapBuffers();
}

void reshape(int width, int height) {
  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0, width / (float)height, 0.1, 100.0);
}
void idle() { display(); }

void keyboard(unsigned char key, int x, int y) {
  switch (key) {
  case ' ':
    if (Breakout.State == dtk::SCENE_PAUSE) {
      Breakout.State = dtk::SCENE_ACTIVE;
      break;
    }
    if (Breakout.State == dtk::SCENE_ACTIVE) {
      Breakout.State = dtk::SCENE_PAUSE;
      break;
    }
  case 27:
    exit(0);
    break;
  default:
    break;
  }
}

void gl_vis(int argc, char *argv[]) {
  glutInit(&argc, argv);
  glutInitWindowSize(800, 600);
  glutInitWindowPosition(50, 50);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
  glutCreateWindow("SimplePhysicsEngine-ST-demo3d");
  glutDisplayFunc(&display);
  glutReshapeFunc(&reshape);
  glutKeyboardFunc(&keyboard);
  Breakout.Init();
  glutIdleFunc(&idle);
  // glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
  glutMainLoop();
}

int main(int argc, char *argv[]) {

  // freeglut可视化
  gl_vis(argc, argv);
  // vtk可视化
  // vtk_vis();
  return 0;
}