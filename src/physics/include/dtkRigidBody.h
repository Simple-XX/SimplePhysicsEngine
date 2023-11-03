
/**
 * @file dtkRigidBody.h
 * @brief dtkRigidBody 头文件
 * @author tom (https://github.com/TOMsworkspace)
 * @version 1.0
 * @date 2021-08-05
 * @copyright MIT LICENSE
 * https://github.com/Simple-XX/SimplePhysicsEngine
 * @par change log:
 * <table>
 * <tr><th>Date<th>Author<th>Description
 * <tr><td>2021-08-05<td>tom<td>创建文件
 * <tr><td>2023-10-31<td>Zone.N<td>迁移到 doxygen
 * </table>
 */

#ifndef SIMPLEPHYSICSENGINE_DTKRIGIBODY_H
#define SIMPLEPHYSICSENGINE_DTKRIGIBODY_H

#include <memory>
#include <vector>

#include "dtkMatrix.h"
#include "dtkTx.h"
// #include "../dtk/dtkIDTypes.h"

namespace dtk {

// 刚体
/**
 * @class <dtkRigidBody>
 * @brief 刚体
 * @author <tom>
 * @note
 * 刚体
 */
using vertex_list = std::vector<dtkDouble2>;
using index_list = std::vector<int>;
class dtkRigidBody {
public:
  using ptr = std::shared_ptr<dtkRigidBody>;
  using vertex_list = std::vector<dtkDouble2>;

  dtkRigidBody(unsigned int id, double mass);
  virtual ~dtkRigidBody() = default;

  bool can_collide(const dtkRigidBody &other) const;
  void update_impulse(const dtkDouble2 &impulse, const dtkDouble2 &r);
  void update_force(const dtkDouble2 &gravity, double dt);

  dtkDouble2 local_to_world(const dtkDouble2 &) const;

  unsigned int get_id() const;

  double get_mass() const;
  void set_mass(double);

  double get_inv_mass() const;

  double get_inertia() const;
  void set_inertia(double);

  double get_inv_inertia() const;

  const dtkDouble2 &get_centroid() const;
  void set_centroid(const dtkDouble2 &);

  const dtkDouble2 &get_position() const;
  void set_position(const dtkDouble2 &);

  const dtkMatrix22 &get_rotation() const;
  void set_rotation(const dtkMatrix22 &);

  const dtkDouble2 &get_velocity() const;
  void set_velocity(const dtkDouble2 &);

  double get_angular_velocity() const;
  void set_angular_velocity(double);

  const dtkDouble2 &get_force() const;
  void set_force(const dtkDouble2 &);

  double get_torque() const;
  void set_torque(double);

  double get_friction() const;
  void set_friction(double);

protected:
  // DISALLOW_COPY_AND_ASSIGN(dtkRigidBody)

  dtkRigidBody(const dtkRigidBody &) = delete;
  const dtkRigidBody &operator=(const dtkRigidBody &) = delete;

  unsigned int mId;        /**< id */
  double mMass;            /**< 质量 */
  double invMass;          /**< 质量倒数（缓存，用于运算） */
  double mInertia;         /**< 转动惯量 */
  double invInertia;       /**< 转动惯性倒数（缓存，用于运算） */
  dtkDouble2 mCentroid;    /**< 重心 */
  dtkDouble2 mPosition;    /**< 位置 */
  dtkMatrix22 mRotation;   /**< 旋转矩阵 */
  dtkDouble2 mVelocity;    /**< 速度 */
  double mAngularVelocity; /**< 角速度 */
  dtkDouble2 mForce;       /**< 受力 */
  double mTorque;          /**< 扭矩 */
  double mFriction;        /**< 摩擦力 */
};

// 凸多边形几何刚体
/**
 * @class <dtkRigidBody>
 * @brief 凸多边形几何刚体
 * @author <tom>
 * @note
 * 凸多边形几何刚体
 */
class dtkPolygonRigidBody : public dtkRigidBody {
public:
  using ptr = std::shared_ptr<dtkPolygonRigidBody>;

  dtkPolygonRigidBody(unsigned int id, double mass,
                      const vertex_list &vertices);

  size_t count() const;

  dtkDouble2 operator[](size_t idx) const;
  dtkDouble2 edge(size_t idx) const;

  double SAT(size_t &idx, const dtkPolygonRigidBody &other) const;
  double min_separating_axis(size_t &idx,
                             const dtkPolygonRigidBody &other) const;
  bool isConcave() { return mIsConcave; }
  std::vector<index_list> mPolygonList; // 子多边形列表

  /// <summary>
  /// 通过比较相邻两条边的叉乘结果，判断多边形是否为凹多边形
  /// </summary>
  /// <param name="pVertices">待判断的多边形顶点列表</param>
  /// <returns></returns>
  bool isConcavePolygon(index_list iVertices);

  /// <summary>
  /// 将凹多边形分割为多个凸多边形(旋转分割法)，并移除掉退化三角形
  /// </summary>
  /// <param name="pVertices">多边形顶点列表</param>
  /// <returns>分割好的多边形</returns>
  std::vector<index_list> decomposeConcavePolygon(index_list pVertices);

  /// <summary>
  /// 将凹多边形分割为多个凸多边形(旋转分割法)
  /// </summary>
  /// <param name="pVertices">多边形顶点列表</param>
  /// <returns>分割好的多边形</returns>
  std::vector<index_list> divideConcavePolygon(index_list iVertices);

protected:
  // DISALLOW_COPY_AND_ASSIGN(dtkPolygonRigidBody)

  dtkPolygonRigidBody(const dtkPolygonRigidBody &) = delete;
  const dtkPolygonRigidBody &operator=(const dtkPolygonRigidBody &) = delete;

  vertex_list mVertices;

  // 凸多边形
  bool mIsConcave; // 是否为凸多边形
};

} // namespace dtk

#endif /* SIMPLEPHYSICSENGINE_DTKRIGIBODY_H */
