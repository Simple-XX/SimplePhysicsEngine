
/**
 * @file FemSimulation.h
 * @brief FemSimulation 头文件
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
 * @Date: 2021-09-03 16:12:05
 * @Last Modified by: tom: https://github.com/TOMsworkspace
 * @Last Modified time: 2021-09-03 17:22:45
 */

#ifndef SIMPLEPHYSICSENGINE_FEMSIMULATION_H
#define SIMPLEPHYSICSENGINE_FEMSIMULATION_H

#include <iostream>
#include <memory>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <dtkJoint.h>
#include <dtkScene.h>

#include "CollisionPair.h"
#include "Constants.h"
#include "Mesh.h"
#include "SPHSolver.h"

struct Circle2 {
  double x, y, radius;
};

class FemSimulation {
public:
  FemSimulation(const dtk::dtkDouble2 &gravity);
  ~FemSimulation() = default;

  // 刚体部分
  using body_list = std::vector<dtk::dtkRigidBody::ptr>;
  using mesh_list = std::vector<Mesh::ptr>;
  using joint_list = std::vector<dtk::dtkJoint::ptr>;
  using pair_list = std::unordered_map<uint32_t, dtk::CollisionPair::ptr>;
  using sph_list = std::vector<SPHSolver::ptr>;

  void add(dtk::dtkRigidBody::ptr body);
  void add(dtk::dtkJoint::ptr joint);
  void add(Mesh::ptr mesh);
  void add(SPHSolver::ptr sph);
  const dtk::dtkDouble2 &get_gravity() const;
  const body_list &get_bodies() const;
  const joint_list &get_joints() const;
  const pair_list &get_arbiters() const;
  const mesh_list &get_meshes() const;
  const sph_list &get_sphs() const;

  void move(const dtk::dtkDouble2 &v);

  void clear();
  void step(double dt);

  bool is_pause() const;
  void set_pause(bool pause);

private:
  Circle2 sphere;
  Eigen::Vector2f spherecenter;
  // 刚体部分

  // 是否暂停
  bool _pause{false};
  // 重力
  dtk::dtkDouble2 _gravity;
  size_t _iterations{10};

  // 刚体列表
  body_list _bodies;
  // 关节数组
  joint_list _joints;
  // 碰撞检测对列表
  pair_list _arbiters;
  mesh_list _meshes;
  sph_list _sphs;
};

class dtkFactory {
public:
  static dtk::dtkPolygonRigidBody::ptr
  make_box(double mass, double width, double height,
           const dtk::dtkDouble2 &position = dtk::dtkDouble2());
  static dtk::dtkPolygonRigidBody::ptr
  make_polygon(double mass,
               const dtk::dtkPolygonRigidBody::vertex_list &vertices,
               const dtk::dtkDouble2 &position = dtk::dtkDouble2());
  static dtk::dtkPolygonRigidBody::ptr make_fence(FemSimulation &world);
  static dtk::CollisionPair::ptr
  make_arbiter(dtk::dtkRigidBody::ptr a, dtk::dtkRigidBody::ptr b,
               const dtk::dtkDouble2 &normal,
               const dtk::CollisionPair::contact_list &contacts =
                   dtk::CollisionPair::contact_list());
  static dtk::dtkRevoluteJoint::ptr
  make_revolute_joint(dtk::dtkRigidBody::ptr a, dtk::dtkRigidBody::ptr b,
                      const dtk::dtkDouble2 &anchor);
  static Mesh::ptr
  make_mesh(int n_x_node, int n_y_node,
            const dtk::dtkDouble2 &position = dtk::dtkDouble2());
  static SPHSolver::ptr make_sph();
};

#endif /* SIMPLEPHYSICSENGINE_FEMSIMULATION_H */
