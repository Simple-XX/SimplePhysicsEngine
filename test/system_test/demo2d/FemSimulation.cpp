
/**
 * @file FemSimulation.cpp
 * @brief FemSimulation 实现
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

/*
 * @Author: tom: https://github.com/TOMsworkspace
 * @Date: 2021-09-03 15:32:52
 * @Last Modified by: tom: https://github.com/TOMsworkspace
 * @Last Modified time: 2021-09-03 19:17:13
 */

#include <GL/freeglut.h>

#include "FemSimulation.h"

float deltat = 0.001;

int iterate_time = 3;

FemSimulation::FemSimulation(const dtk::dtkDouble2 &gravity)
    : spherecenter(0.4, 0.5), _gravity(gravity) {
  sphere.x = spherecenter[0];
  sphere.y = spherecenter[1];
}

void FemSimulation::add(dtk::dtkRigidBody::ptr body) {
  _bodies.push_back(body);
}

void FemSimulation::add(dtk::dtkJoint::ptr joint) { _joints.push_back(joint); }

void FemSimulation::add(Mesh::ptr mesh) {
  mesh->Init();
  _meshes.push_back(mesh);
}

void FemSimulation::add(SPHSolver::ptr sph) { _sphs.push_back(sph); }

const dtk::dtkDouble2 &FemSimulation::get_gravity() const { return _gravity; }

const FemSimulation::body_list &FemSimulation::get_bodies() const {
  return _bodies;
}

const FemSimulation::joint_list &FemSimulation::get_joints() const {
  return _joints;
}

const FemSimulation::pair_list &FemSimulation::get_arbiters() const {
  return _arbiters;
}

const FemSimulation::mesh_list &FemSimulation::get_meshes() const {
  return _meshes;
}

const FemSimulation::sph_list &FemSimulation::get_sphs() const { return _sphs; }

void FemSimulation::move(const dtk::dtkDouble2 &v) {
  for (auto &body : _bodies) {
    if (body->get_inv_mass() > 0)
      body->update_impulse(v * body->get_inv_mass(), dtk::dtkDouble2());
  }
}

void FemSimulation::step(double dt) {
  if (_pause)
    return;
  // 碰撞检测
  int body_cnt = _bodies.size();
  for (size_t i = 0; i < body_cnt; ++i) {
    for (size_t j = i + 1; j < body_cnt; ++j) {
      auto a = std::dynamic_pointer_cast<dtk::dtkPolygonRigidBody>(_bodies[i]);
      auto b = std::dynamic_pointer_cast<dtk::dtkPolygonRigidBody>(_bodies[j]);
      if (!a->can_collide(*b)) {
        continue;
      }
      uint32_t id;
      auto arbiter = dtk::CollisionPair::is_collide_rr(a, b, id);
      auto iter = _arbiters.find(id);
      if (arbiter == nullptr) {
        if (iter != _arbiters.end()) {
          _arbiters.erase(id);
        }
      } else if (iter == _arbiters.end()) {
        _arbiters[id] = arbiter;
      } else {
        auto &old_arbiter = iter->second;
        arbiter->update(*old_arbiter);
        _arbiters[id] = arbiter;
      }
    }
  }
  for (auto &kv : _arbiters) {
    kv.second->pre_step(dt);
  }
  for (auto &joint : _joints) {
    joint->pre_step(dt);
  }

  // 更新动量
  for (size_t i = 0; i < _iterations; ++i) {
    for (auto &kv : _arbiters) {
      kv.second->update_impulse();
    }
    for (auto &joint : _joints) {
      joint->update_impulse();
    }
  }

  // 更新重力
  for (auto &body : _bodies) {
    body->update_force(_gravity, dt);
  }

  // 更新mesh运动
  for (auto &mesh : _meshes) {
    for (int i = 0; i < iterate_time; ++i) {
      for (auto &body : _bodies) {
        auto poly_body =
            std::dynamic_pointer_cast<dtk::dtkPolygonRigidBody>(body);
        dtk::CollisionPair::do_collision_mr(mesh, poly_body);
      }
      mesh->compute_force();

      for (int j = 0; j < (*mesh).n_node_; ++j) {
        (*mesh).points_v_[j] =
            ((*mesh).points_v_[j] +
             ((*mesh).points_force_[j] / (*mesh).node_mass_) * deltat) *
            exp(deltat * -3);

        (*mesh).points_[j] += deltat * (*mesh).points_v_[j];
      }
    }
    mesh->updateShell();
  }
  // 更新sph
  for (auto &sph : _sphs) {
    sph->update(Constants::TIMESTEP);
  }
}

void FemSimulation::clear() {
  _arbiters.clear();
  _joints.clear();
  _bodies.clear();
  _meshes.clear();
  _sphs.clear();
}

bool FemSimulation::is_pause() const { return _pause; }

void FemSimulation::set_pause(bool pause) { _pause = pause; }

// 工厂
static uint16_t global_id = 1;

dtk::dtkPolygonRigidBody::ptr
dtkFactory::make_box(double mass, double width, double height,
                     const dtk::dtkDouble2 &position) {
  // 四边形
  // 注意顶点呈逆时针
  dtk::dtkPolygonRigidBody::vertex_list vertices = {{width / 2, height / 2},
                                                    {-width / 2, height / 2},
                                                    {-width / 2, -height / 2},
                                                    {width / 2, -height / 2}};
  auto body =
      std::make_shared<dtk::dtkPolygonRigidBody>(global_id++, mass, vertices);
  body->set_position(position);
  return body;
}

dtk::dtkPolygonRigidBody::ptr
dtkFactory::make_polygon(double mass,
                         const dtk::dtkPolygonRigidBody::vertex_list &vertices,
                         const dtk::dtkDouble2 &position) {
  auto body =
      std::make_shared<dtk::dtkPolygonRigidBody>(global_id++, mass, vertices);
  body->set_position(position);
  return body;
}

dtk::dtkPolygonRigidBody::ptr dtkFactory::make_fence(FemSimulation &world) {
  auto ground = make_box(dtk::inf, 20, 1, {0, -0.5});
  world.add(ground);
  world.add(make_box(dtk::inf, 20, 1, {0, 16.5}));
  world.add(make_box(dtk::inf, 1, 18, {-9.5, 8}));
  world.add(make_box(dtk::inf, 1, 18, {9.5, 8}));
  return ground;
}

dtk::CollisionPair::ptr
dtkFactory::make_arbiter(dtk::dtkRigidBody::ptr a, dtk::dtkRigidBody::ptr b,
                         const dtk::dtkDouble2 &normal,
                         const dtk::CollisionPair::contact_list &contacts) {
  return std::make_shared<dtk::CollisionPair>(a, b, normal, contacts);
}

dtk::dtkRevoluteJoint::ptr
dtkFactory::make_revolute_joint(dtk::dtkRigidBody::ptr a,
                                dtk::dtkRigidBody::ptr b,
                                const dtk::dtkDouble2 &anchor) {
  return std::make_shared<dtk::dtkRevoluteJoint>(a, b, anchor);
}

Mesh::ptr dtkFactory::make_mesh(int n_x_node, int n_y_node,
                                const dtk::dtkDouble2 &position) {
  return std::make_shared<Mesh>(global_id++, n_x_node, n_y_node,
                                Eigen::Vector2f(position.x, position.y));
}

SPHSolver::ptr dtkFactory::make_sph() { return std::make_shared<SPHSolver>(); }
