
/**
 * @file dtkCollisionDetectBasic.h
 * @brief  dtkCollisionDetectBasic 头文件
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

#ifndef SIMPLEPHYSICSENGINE_DTKCOLLISIONDETECTBASIC_H
#define SIMPLEPHYSICSENGINE_DTKCOLLISIONDETECTBASIC_H

#include "dtkCollisionDetectNode.h"
#include "dtkCollisionDetectPrimitive.h"
#include "dtkGraphicsKernel.h"
#include "dtkIntersectTest.h"

namespace dtk {
/**
 * @class <dtkCollisionDetectBasic>
 * @brief 碰撞检测中的相交测试。
 * @author <>
 * @note
 * 碰撞检测中的相交测试。两个图元进行相交测试或是两组图元进行相交测试。得到由dtkIntersectTest::IntersectResult定义的相交结果。
 */
class dtkCollisionDetectBasic {
public:
  typedef dtkIntersectTest::IntersectResult
      IntersectResult; /**< 相交测试结果 */

public:
  /**
   * @brief 两个图元进行相交测试
   * @param[in]    pri_1 : 图元1
   * @param[in]	pri_2 : 图元2
   * @param[in]	result : 指向相交测试的具体结果的智能指针引用
   * @param[in]	self : 自相交
   * @param[in]	ignore_extend : 相交测试间隔
   * @note	间隔是存在于图元间的距离，只要小于间隔就视为相交。
   * @return
   *	true 相交 \n
   *	false 不相交 \n
   */
  static bool DoIntersect(dtkCollisionDetectPrimitive *pri_1,
                          dtkCollisionDetectPrimitive *pri_2,
                          IntersectResult::Ptr &result, bool self = false,
                          bool ignore_extend = false);

  static bool DoIntersect(const dtkCollisionDetectNode *node_1,
                          const dtkCollisionDetectNode *node_2);
};

typedef dtkCollisionDetectBasic CDBasic;
}; // namespace dtk

#endif /* SIMPLEPHYSICSENGINE_DTKCOLLISIONDETECTBASIC_H */
