
/**
 * @file dtkCollisionDetectHierarchyKDOPS.h
 * @brief  dtkCollisionDetectHierarchyKDOPS 头文件
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

#ifndef SIMPLEPHYSICSENGINE_DTKCOLLISIONDETECTHIERARCHYKDOPS_H
#define SIMPLEPHYSICSENGINE_DTKCOLLISIONDETECTHIERARCHYKDOPS_H

#include <memory>

#include <boost/utility.hpp>

#include "dtkCollisionDetectHierarchy.h"
#include "dtkCollisionDetectNodeKDOPS.h"

namespace dtk {
/**
 * @class <dtkCollisionDetectHierarchyKDOPS>
 * @brief k-Dops冲突检测算法
 * @author <>
 * @note
 * k-DOPs算法冲突检测树的构建，碰撞检测的执行， 图元的更新等。
 */
class dtkCollisionDetectHierarchyKDOPS : public dtkCollisionDetectHierarchy {
public:
  typedef std::shared_ptr<dtkCollisionDetectHierarchyKDOPS> Ptr;

  static dtkCollisionDetectHierarchyKDOPS::Ptr New(size_t half_k) {
    return dtkCollisionDetectHierarchyKDOPS::Ptr(
        new dtkCollisionDetectHierarchyKDOPS(half_k));
  }

public:
  virtual ~dtkCollisionDetectHierarchyKDOPS();

  /**
   * @brief		构建k-Dpos冲突检测树
   */
  // 构建冲突检测树
  void Build();

  /**
   * @brief		在图元更新时，重新构建冲突检测树
   */
  // 重新构建冲突检测树
  void Rebuild();

  /**
   * @brief		k-Dops算法更新，更新轴向包围盒,更新图元状态
   */
  // 更新图元包围盒

  void Update();

private:
  dtkCollisionDetectHierarchyKDOPS(size_t half_k);

private:
  size_t mHalfK; /**< k-Dops算法轴向包围盒维度 */
};
} // namespace dtk

#endif /* SIMPLEPHYSICSENGINE_DTKCOLLISIONDETECTHIERARCHYKDOPS_H */
