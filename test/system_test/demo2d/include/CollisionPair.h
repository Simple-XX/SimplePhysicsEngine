
/**
 * @file CollisionPair.h
 * @brief CollisionPair 头文件
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

#ifndef SIMPLEPHYSICSENGINE_COLLISIONPAIR_H
#define SIMPLEPHYSICSENGINE_COLLISIONPAIR_H

#define MAKE_ID(a, b) (((a) < (b)) ? (((a) << 16) | (b)) : (((b) << 16) | (a)))

#include <array>
#include <memory>

#include <dtkRigidBody.h>

#include "mesh.h"

// 接触点
struct ccontact {
  // 位置
  dtk::dtkDouble2 position;
  dtk::dtkDouble2 ra, rb;
  std::array<bool, 2> from_a;
  std::array<size_t, 2> indices;
  double separation;
  double pn{0};
  double pt{0};
  double bias{0};
  double mass_normal{0};
  double mass_tangent{0};

  ccontact(const dtk::dtkPolygonRigidBody &b, size_t idx);

  bool operator==(const ccontact &other) const;
  bool operator!=(const ccontact &other) const;
};

// 碰撞检测对
class CollisionPair {
public:
  using contact_list = std::vector<ccontact>;
  using ptr = std::shared_ptr<CollisionPair>;

  CollisionPair(dtk::dtkRigidBody::ptr a, dtk::dtkRigidBody::ptr b,
                const dtk::dtkDouble2 &normal, const contact_list &contacts);

  const contact_list &get_contacts() const;
  const dtk::dtkDouble2 &get_normal() const;

  void pre_step(double dt);
  void update_impulse();
  void update(const CollisionPair &old_arbiter);

  void add_contact(const ccontact &contact);

  static ptr is_collide_rr(dtk::dtkPolygonRigidBody::ptr &pa,
                           dtk::dtkPolygonRigidBody::ptr &pb, uint32_t &id);
  static void do_collision_mr(Mesh::ptr &pa, dtk::dtkPolygonRigidBody::ptr &pb);

private:
  // 参与碰撞检测的两个刚体
  std::weak_ptr<dtk::dtkRigidBody> _a, _b;
  // 法向量
  dtk::dtkDouble2 _normal;
  // 接触点列表
  contact_list _contacts;
};

#endif /* SIMPLEPHYSICSENGINE_COLLISIONPAIR_H */
