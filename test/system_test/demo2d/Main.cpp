
/**
 * @file Main.cpp
 * @brief Main 实现
 * @author Zone.N (Zone.Niuzh@hotmail.com)
 * @version 1.0
 * @date 2023-10-31
 * @copyright MIT LICENSE
 * https://github.com/Simple-XX/SimplePhysicsEngine
 * @par change log:
 * <table>
 * <tr><th>Date<th>Author<th>Description
 * <tr><td>2023-10-31<td>Zone.N<td>迁移到 doxygen
 * </table>
 */

#include <chrono>
#include <cmath>
#include <iostream>

#include <GL/freeglut.h>

#include "dtkFemSimulation.h"
#include "dtkScene.h"

static auto last_clock = std::chrono::high_resolution_clock::now();

// The Width of the screen
const unsigned int SCREEN_WIDTH = 800;
// The height of the screen
const unsigned int SCREEN_HEIGHT = 600;
static dtkFemSimulation world({0, -9.8});

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

static void draw_divided_polys(dtk::dtkPolygonRigidBody &body) {
  int len1 = body.mPolygonList.size();
  for (int i = 0; i < len1; i++) {
    glBegin(GL_LINE_LOOP);
    int len2 = body.mPolygonList[i].size();
    for (int j = 0; j < len2; j++) {
      glVertex2d(body[body.mPolygonList[i][j]].x,
                 body[body.mPolygonList[i][j]].y);
    }
    glEnd();
  }
}

static void draw_body(const dtk::dtkPolygonRigidBody &body) {
  if (std::isinf(body.get_mass())) {
    glColor3f(1.0f, 1.0f, 1.0f);
  } else {
    glColor3f(0.8f, 0.8f, 0.0f);
  }
  glBegin(GL_LINE_LOOP);
  for (size_t i = 0; i < body.count(); ++i) {
    auto point = body.local_to_world(body[i]);
    glVertex2d(point.x, point.y);
  }
  glEnd();
  if (!std::isinf(body.get_mass())) {
    auto &pos = body.get_position();
    auto v = body.local_to_world(body.get_velocity() * 0.2);
    glBegin(GL_LINES);
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex2d(pos.x, pos.y);
    glVertex2d(v.x, v.y);
    glEnd();
    glPointSize(3.0f);
    glBegin(GL_POINTS);
    glVertex2d(pos.x, pos.y);
    glEnd();
  }
}

static void draw_mesh(const dtkMesh &mesh) {
  glColor3f(0.8f, 0.8f, 0.0f);
  glBegin(GL_LINES);
  for (int i = 0; i < mesh.n_fem_element_; ++i) {

    for (int j = 0; j < 3; ++j) {
      int a = mesh.mesh_table_[i][j];
      int b = mesh.mesh_table_[i][(j + 1) % 3];

      // draw line from a to b;
      glVertex2f(mesh.points_[a][0], mesh.points_[a][1]);
      glVertex2f(mesh.points_[b][0], mesh.points_[b][1]);
    }
  }
  glEnd();
}

static void draw_mesh_shell(const dtkMesh &mesh) {
  glColor3f(0.8f, 0.0f, 0.0f);

  glBegin(GL_LINE_LOOP);
  for (int i = 0; i < mesh.shell->count(); ++i) {
    glVertex2f((*mesh.shell)[i].x, (*mesh.shell)[i].y);
  }
  glEnd();

  // draw_divided_polys(*(mesh.shell));
}

static void draw_joint(const dtk::dtkRevoluteJoint &joint) {
  auto centroid_a =
      joint.get_a()->local_to_world(joint.get_a()->get_centroid());
  auto anchor_a = joint.world_anchor_a();
  auto centroid_b =
      joint.get_b()->local_to_world(joint.get_b()->get_centroid());
  auto anchor_b = joint.world_anchor_b();

  glColor3f(0.6f, 0.6f, 0.6f);
  glBegin(GL_LINES);
  if (!std::isinf(joint.get_a()->get_mass())) {
    glVertex2d(centroid_a.x, centroid_a.y);
    glVertex2d(anchor_a.x, anchor_a.y);
  }
  if (!std::isinf(joint.get_b()->get_mass())) {
    glVertex2d(centroid_b.x, centroid_b.y);
    glVertex2d(anchor_b.x, anchor_b.y);
  }
  glEnd();
}

static void draw_arbiter(const dtk::dtkCollisionPair::ptr &pair) {
  auto &contacts = pair->get_contacts();
  for (auto &contact : contacts) {
    auto pos = contact.position;
    auto ra = pos + dtk::normalize(contact.ra) * 0.2;
    auto rb = pos + dtk::normalize(contact.rb) * 0.2;
    glColor3f(0.2f, 0.2f, 1.0f);
    glBegin(GL_LINES);
    glVertex2d(pos.x, pos.y);
    glVertex2d(ra.x, ra.y);
    glVertex2d(pos.x, pos.y);
    glVertex2d(rb.x, rb.y);
    glEnd();
    glColor3f(1.0f, 0.2f, 0.2f);
    glPointSize(2.0f);
    glBegin(GL_POINTS);
    glVertex2d(pos.x, pos.y);
    glEnd();
  }
}

void drawParticle(float x, float y, float s) {
  glBegin(GL_POLYGON);
  glVertex2f(x, y + s);
  glVertex2f(x - s, y);
  glVertex2f(x, y - s);
  glVertex2f(x + s, y);
  glEnd();
}

void draw_sph(SPHSolver &sph) {
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslatef(-1.5f, -1.0f, -5.0f);
  // for each (auto & particle in sph.particles)
  for (auto &particle : sph.particles) {
    glColor3f(0.5, 1.0, 1.0);
    drawParticle(particle.position[0], particle.position[1], 0.01);
  }
}

double random(double low, double high) {
  return 1.0 * rand() / RAND_MAX * (high - low) + low;
}

static void test_polygon() {
  dtkFactory::make_fence(world);
  world.add(dtkFactory::make_polygon(200, {{-4, 0}, {4, 0}, {4, 1}, {-4, 1}},
                                     {0, 0}));
  world.add(dtkFactory::make_polygon(200, {{-1, -2}, {1, -2}, {1, 2}, {-1, 2}},
                                     {0, 4}));
  world.add(dtkFactory::make_polygon(200, {{2, -1}, {0, 1}, {-2, -1}, {0, 0}},
                                     {0, 10}));
}

static void test_stack() {
  dtkFactory::make_fence(world);
  for (int i = 0; i < 10; ++i) {
    double x = random(-0.1 * i, 0.1 * i);
    auto body = dtkFactory::make_box(1, 1, 1, {x, 0.51f + 1.05f * i});
    body->set_friction(0.2);
    world.add(body);
  }
}

static void test_pyramid() {
  dtkFactory::make_fence(world);
  dtk::dtkDouble2 x(-6.0f, 0.75f);
  dtk::dtkDouble2 y;
  int n = 10;
  for (int i = 0; i < n; ++i) {
    y = x;
    for (int j = i; j < n; ++j) {
      auto body = dtkFactory::make_box(10, 1, 1, y);
      body->set_friction(0.2);
      world.add(body);
      y += dtk::dtkDouble2(1.125f, 0.0f);
    }
    x += dtk::dtkDouble2(0.5625f, 1.5f);
  }
}

static void test_joint() {
  auto ground = dtkFactory::make_box(dtk::inf, 100, 20, {0, -10});
  world.add(ground);

  auto box1 = dtkFactory::make_box(500, 1, 1, {13.5, 11});
  world.add(box1);
  auto joint1 = dtkFactory::make_revolute_joint(ground, box1, {4.5, 11});
  world.add(joint1);

  for (size_t i = 0; i < 5; ++i) {
    auto box2 = dtkFactory::make_box(100, 1, 1, {3.5f - i, 2});
    world.add(box2);
    auto joint2 = dtkFactory::make_revolute_joint(ground, box2, {3.5f - i, 11});
    world.add(joint2);
  }
}

static void test_chain() {
  auto ground = dtkFactory::make_box(dtk::inf, 100, 20, {0, -10});
  ground->set_friction(0.4);
  world.add(ground);

  const double mass = 10.0f;
  const double y = 12.0f;
  auto last = ground;
  for (int i = 0; i < 15; ++i) {
    auto box = dtkFactory::make_box(mass, 0.75, 0.25, {0.5f + i, y});
    box->set_friction(0.4);
    world.add(box);
    auto joint =
        dtkFactory::make_revolute_joint(last, box, dtk::dtkDouble2(i, y));
    world.add(joint);
    last = box;
  }
}

static void test_mesh() {
  dtkFactory::make_fence(world);
  auto mesh = dtkFactory::make_mesh(15, 3, dtk::dtkDouble2(0.0, 6.0));
  world.add(mesh);
  auto box = dtkFactory::make_polygon(200, {{-1, 0}, {1, 0}, {1, 2}, {-1, 2}},
                                      {0.2, 0});
  world.add(box);
}

static void test_sph() {
  auto sph = dtkFactory::make_sph();
  world.add(sph);
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

  draw_text(5, 20, "dtk @RigidBody simulation");

  draw_text(5, 40, "Push [1-7] to switch scene");
  draw_text(w - 150, h - 20, "refer: apollonia");

  if (world.is_pause())
    draw_text(5, h - 20, "dt: %.2f ms PAUSED", dt * 1000);
  else
    draw_text(5, h - 20, "dt: %.2f ms", dt * 1000);

  world.step(std::min(dt, 0.01));

  for (auto &body : world.get_bodies()) {
    draw_body(*std::dynamic_pointer_cast<dtk::dtkPolygonRigidBody>(body).get());
  }

  for (auto &joint : world.get_joints()) {
    draw_joint(*std::dynamic_pointer_cast<dtk::dtkRevoluteJoint>(joint).get());
  }

  for (auto &arbiter : world.get_arbiters()) {
    draw_arbiter(arbiter.second);
  }

  for (auto &mesh : world.get_meshes()) {
    draw_mesh(*std::dynamic_pointer_cast<dtkMesh>(mesh).get());
    draw_mesh_shell(*std::dynamic_pointer_cast<dtkMesh>(mesh).get());
  }

  for (auto &sph : world.get_sphs()) {
    draw_sph(*std::dynamic_pointer_cast<SPHSolver>(sph).get());
  }

  glutSwapBuffers();
}

void reshape(int width, int height) {
  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0, width / (float)height, 0.1, 100.0);
}

void mouse(int button, int state, int x, int y) {}

void move_pos(const dtk::dtkDouble2 &v) { world.move(v); }

void keyboard(unsigned char key, int x, int y) {
  switch (key) {
  case '1':
    world.clear();
    test_polygon();
    break;
  case '2':
    world.clear();
    test_stack();
    break;
  case '3':
    world.clear();
    test_pyramid();
    break;
  case '4':
    world.clear();
    test_joint();
    break;
  case '5':
    world.clear();
    test_chain();
    break;
  case '6':
    world.clear();
    test_mesh();
    break;
  case '7':
    world.clear();
    test_sph();
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
    /// @todo not found
    // glutLeaveMainLoop();
    break;
  default:
    break;
  }
}

void motion(int x, int y) {}

void special(int key, int x, int y) {}

void idle() { display(); }

int main(int argc, char *argv[]) {
  glutInit(&argc, argv);
  glutInitWindowSize(800, 600);
  glutInitWindowPosition(50, 50);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
  glutCreateWindow("Physics Engine -- dtk");
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
