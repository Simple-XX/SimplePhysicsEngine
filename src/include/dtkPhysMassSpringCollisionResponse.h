
/**
 * @file dtkPhysMassSpringCollisionResponse.h
 * @brief  dtkPhysMassSpringCollisionResponse 头文件
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

#ifndef SIMPLEPHYSICSENGINE_DTKPHYSMASSSPRINGCOLLISIONRESPONSE_H
#define SIMPLEPHYSICSENGINE_DTKPHYSMASSSPRINGCOLLISIONRESPONSE_H

#include <map>
#include <memory>
#include <vector>

#include <boost/utility.hpp>

#include "dtkIntersectTest.h"
#include "dtkPhysMassSpring.h"

namespace dtk {
class dtkPhysMassSpringCollisionResponse : public boost::noncopyable {
public:
  enum ResponseType {
    TRIANGLE_SEGMENT = 0, /**< 三角形与线段相交 */
    SEGMENT_SEGMENT,      /**< 线段与线段相交 */
    TRIANGLE_TRIANGLE     /**< 三角形与三角形相交 */
  };

public:
  typedef std::shared_ptr<dtkPhysMassSpringCollisionResponse> Ptr;

  static Ptr New() { return Ptr(new dtkPhysMassSpringCollisionResponse()); }

public:
  ~dtkPhysMassSpringCollisionResponse();

  void
  Update(double timeslice,
         std::vector<dtkIntersectTest::IntersectResult::Ptr> &intersectResults,
         const std::vector<dtkInterval<int>> &avoid_1,
         const std::vector<dtkInterval<int>> &avoid_2, double stiffness);

  void AddPierceSegment(dtkID majorID, dtkID minorID) {
    mPierceSegments.push_back(dtkID2(majorID, minorID));
  }

  void SetMassSpring(dtkID i, dtkPhysMassSpring::Ptr massSpring) {
    mMassSprings[i] = massSpring;
  }

  void RemoveMassSpring(dtkID i) { mMassSprings.erase(i); }

  dtkPhysMassSpring::Ptr GetMassSpring(dtkID majorID) {
    assert(mMassSprings.find(majorID) != mMassSprings.end());
    return mMassSprings[majorID];
  }

  std::vector<dtkIntersectTest::IntersectResult::Ptr> &GetPiercingResults() {
    return mPiercingResults;
  }

private:
  dtkPhysMassSpringCollisionResponse();

private:
  std::map<dtkID, dtkPhysMassSpring::Ptr> mMassSprings;

  std::vector<dtkID2> mPierceSegments;

  std::vector<dtkIntersectTest::IntersectResult::Ptr> mPiercingResults;
};
} // namespace dtk

#endif /* SIMPLEPHYSICSENGINE_DTKPHYSMASSSPRINGCOLLISIONRESPONSE_H */
