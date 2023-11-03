
/**
 * @file dtkJoint.h
 * @brief dtkJoint 头文件
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

#ifndef SIMPLEPHYSICSENGINE_DTKJOINT_H
#define SIMPLEPHYSICSENGINE_DTKJOINT_H

#include <memory>

#include "dtkMatrix.h"
#include "dtkRigidBody.h"
#include "dtkTx.h"

namespace dtk {

/**
 * @class <dtkJoint>
 * @brief 刚体的关节
 * @author <tom>
 * @note
 */
class dtkJoint {
public:
  using ptr = std::shared_ptr<dtkJoint>;

  dtkJoint(dtkRigidBody::ptr a, dtkRigidBody::ptr b);
  virtual ~dtkJoint() = default;

  virtual void pre_step(double dt) = 0;
  virtual void update_impulse() = 0;

  dtkRigidBody::ptr get_a() const;
  void set_a(dtkRigidBody::ptr a);
  dtkRigidBody::ptr get_b() const;
  void set_b(dtkRigidBody::ptr b);

protected:
  dtkJoint(const dtkJoint &) = delete;
  const dtkJoint &operator=(const dtkJoint &) = delete;

  std::weak_ptr<dtkRigidBody> mBodyA, mBodyB; /**< 关节相联结的两个刚体 */
};

/**
 * @class <dtkRevoluteJoint>
 * @brief 连接刚体的旋转关节
 * @author <tom>
 * @note 实现刚体转动，摆动等效果
 */
class dtkRevoluteJoint : public dtkJoint {
public:
  using ptr = std::shared_ptr<dtkJoint>;

  dtkRevoluteJoint(dtkRigidBody::ptr a, dtkRigidBody::ptr b,
                   const dtkDouble2 &anchor);

  void pre_step(double dt) override;
  void update_impulse() override;

  const dtkDouble2 &anchor() const;
  dtkDouble2 world_anchor_a() const;
  dtkDouble2 world_anchor_b() const;

protected:
  dtkRevoluteJoint(const dtkRevoluteJoint &) = delete;
  const dtkRevoluteJoint &operator=(const dtkRevoluteJoint &) = delete;

  dtkDouble2 mAnchor;       /**< 固定位置（世界坐标） */
  dtkDouble2 mLocalAnchorA; /**< 刚体a相对坐标 */
  dtkDouble2 mLocalAnchorB; /**< 刚体b相对坐标 */

  dtkDouble2 mRotateA;  /**< 刚体a旋转角度向量 */
  dtkDouble2 mRotateB;  /**< b旋转角度向量 */
  dtkMatrix22 mMassAll; /**< 总质量 */
  dtkDouble2 mMomentum; /**< 动量 */
  dtkDouble2 mBias;     /**< 修正 */
};
} // namespace dtk

#endif /* SIMPLEPHYSICSENGINE_DTKJOINT_H */
