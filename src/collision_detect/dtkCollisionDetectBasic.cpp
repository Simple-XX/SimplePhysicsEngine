
/**
 * @file dtkCollisionDetectBasic.cpp
 * @brief dtkCollisionDetectBasic 实现
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

#include <algorithm>
#ifdef DTK_DEBUG
#define DTK_COLLISIONDETECTBASIC_DEBUG
#endif // DTK_DEBUG
#ifdef DTK_COLLISIONDETECTBASIC_DEBUG
#include <iostream>
#endif

#include "dtkAssert.h"
#include "dtkCollisionDetectBasic.h"
#include "dtkCollisionDetectNodeKDOPS.h"

using namespace std;

namespace dtk {
// 两个图元进行相交测试
bool dtkCollisionDetectBasic::DoIntersect(dtkCollisionDetectPrimitive *pri_1,
                                          dtkCollisionDetectPrimitive *pri_2,
                                          IntersectResult::Ptr &result,
                                          bool self, bool ignore_extend) {
  if (self) // 自交
  {
    for (dtkID i = 0; i < pri_1->mIDs.size(); i++) {
      for (dtkID j = 0; j < pri_2->mIDs.size(); j++) {
        if (pri_1->mIDs[i] == pri_2->mIDs[j]) {
          return false;
        }
      }
    }
  }

  bool exchanged = false;

  // according primitives' vertex to make geometry object.
  // 根据图元的顶点来制作几何对象。

  const GK::Object &obj_1 = pri_1->GetObject();
  const GK::Object &obj_2 = pri_2->GetObject();

  bool intersected = false;
  double distance = pri_1->GetExtend() + pri_2->GetExtend();
  assert(distance >= 0);
  // ignore_extend represent considering the thickness of the two primitives.
  // ignore_extend 表示考虑两个图元的厚度。

  if (const GK::Triangle3 *tri_1 = CGAL::object_cast<GK::Triangle3>(&obj_1)) {
    if (const GK::Triangle3 *tri_2 = CGAL::object_cast<GK::Triangle3>(&obj_2)) {
      // 两个三角形相交。 考虑间距或者不考虑。

      if (ignore_extend || distance == 0)
        intersected = dtkIntersectTest::DoIntersect(*tri_1, *tri_2, result);
      else
        intersected = dtkIntersectTest::DoDistanceIntersect(
            *tri_1, *tri_2, distance, result,
            max(pri_1->mInvert, pri_2->mInvert));
    } else if (const GK::Segment3 *seg_2 =
                   CGAL::object_cast<GK::Segment3>(&obj_2)) {
      // 一个三角形与一个线段相交。 考虑间距或者不考虑。

      if (ignore_extend || distance == 0)
        intersected = dtkIntersectTest::DoIntersect(*tri_1, *seg_2, result);
      else
        intersected = dtkIntersectTest::DoDistanceIntersect(
            *tri_1, *seg_2, distance, result, pri_2->mInvert);
    } else if (const GK::Sphere3 *sphere =
                   CGAL::object_cast<GK::Sphere3>(&obj_2)) {
      // 一个三角形与一个球相交。 考虑间距或者不考虑。

      if (ignore_extend || distance == 0)
        assert(false); // intersected = dtkIntersectTest::DoIntersect( *tri_1,
                       // *sphere, result );
      else
        intersected = dtkIntersectTest::DoDistanceIntersect(*tri_1, *sphere,
                                                            distance, result);
    }
  } else if (const GK::Segment3 *seg_1 =
                 CGAL::object_cast<GK::Segment3>(&obj_1)) {

    if (const GK::Triangle3 *tri_2 = CGAL::object_cast<GK::Triangle3>(&obj_2)) {
      // 一个三角形与一个线段相交。 考虑间距或者不考虑。

      exchanged = true;
      if (ignore_extend || distance == 0)
        intersected = dtkIntersectTest::DoIntersect(*tri_2, *seg_1, result);
      else
        intersected = dtkIntersectTest::DoDistanceIntersect(
            *tri_2, *seg_1, distance, result, pri_1->mInvert);
    } else if (const GK::Segment3 *seg_2 =
                   CGAL::object_cast<GK::Segment3>(&obj_2)) {
      // 两个线段相交。 考虑间距或者不考虑。

      if (ignore_extend || distance == 0)
        intersected = dtkIntersectTest::DoIntersect(*seg_1, *seg_2, result);
      else
        intersected = dtkIntersectTest::DoDistanceIntersect(*seg_1, *seg_2,
                                                            distance, result);
    }
  } else {
    dtkAssert(false, NOT_IMPLEMENTED);
    return false;
  }

  if (intersected) {
    if (exchanged) {
      result->SetProperty(dtkIntersectTest::INTERSECT_PRIMITIVE_2, pri_1);
      result->SetProperty(dtkIntersectTest::INTERSECT_PRIMITIVE_1, pri_2);
    } else {
      result->SetProperty(dtkIntersectTest::INTERSECT_PRIMITIVE_1, pri_1);
      result->SetProperty(dtkIntersectTest::INTERSECT_PRIMITIVE_2, pri_2);
    }
    pri_1->SetIntersected(true);
    pri_2->SetIntersected(true);
  }

  return intersected;
}

bool dtkCollisionDetectBasic::DoIntersect(
    const dtkCollisionDetectNode *node_1,
    const dtkCollisionDetectNode *node_2) {
  if (const dtkCollisionDetectNodeKDOPS *node_kdops_1 =
          dynamic_cast<const dtkCollisionDetectNodeKDOPS *>(node_1)) {
    if (const dtkCollisionDetectNodeKDOPS *node_kdops_2 =
            dynamic_cast<const dtkCollisionDetectNodeKDOPS *>(
                node_2)) { // kdops 包围盒碰撞检测
      return dtkIntersectTest::DoIntersect(node_kdops_1->GetKDOP(),
                                           node_kdops_2->GetKDOP());
    } else {
      dtkAssert(false, NOT_IMPLEMENTED);
      return false;
    }
  } else {
    dtkAssert(false, NOT_IMPLEMENTED);
    return false;
  }
}
} // namespace dtk
