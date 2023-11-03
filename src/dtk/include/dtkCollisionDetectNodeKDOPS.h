
/**
 * @file dtkCollisionDetectNodeKDOPS.h
 * @brief  dtkCollisionDetectNodeKDOPS 头文件
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

#ifndef SIMPLEPHYSICSENGINE_DTKCOLLISIONDETECTNODEKDOPS_H
#define SIMPLEPHYSICSENGINE_DTKCOLLISIONDETECTNODEKDOPS_H

#include "dtkCollisionDetectNode.h"
#include "dtkConfig.h"
#include "dtkGraphicsKernel.h"

namespace dtk {
class dtkCollisionDetectHierarchyKDOPS;

/**
 * @class <dtkCollisionDetectNode>
 * @brief k-Dops算法冲突检测树结点
 * @author <>
 * @note
 * k-Dops算法冲突检测树结点类，继承于dtkCollisionDetectNode基类。
 */

class dtkCollisionDetectNodeKDOPS : public dtkCollisionDetectNode {
public:
  dtkCollisionDetectNodeKDOPS(dtkCollisionDetectHierarchy *father,
                              size_t half_k);

  ~dtkCollisionDetectNodeKDOPS();

  /**
   * @brief 递归划分k-Dops冲突检测树。
   */

  // 递归划分
  void Split();

  /**
   * @brief 更新k-Dops冲突检测树轴向包围盒。
   */
  // 更新包围盒

  void Update();

  /**
   * @brief 根据图元重心平均值划分为节点为左右分支。
   */
  // 根据图元重心平均值划分为左右分支。

  void SplitRule();

  inline const GK::KDOP &GetKDOP() const { return mKDOP; }

private:
  GK::KDOP mKDOP; /**< 轴向多面体包围盒 */
};
} // namespace dtk

#endif /* SIMPLEPHYSICSENGINE_DTKCOLLISIONDETECTNODEKDOPS_H */
