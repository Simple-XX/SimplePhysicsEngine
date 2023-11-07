
/**
 * @file dtkPhysKnotPlanner.h
 * @brief  dtkPhysKnotPlanner 头文件
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

#ifndef SIMPLEPHYSICSENGINE_DTKPHYSKNOTPLANNER_H
#define SIMPLEPHYSICSENGINE_DTKPHYSKNOTPLANNER_H

#include <map>
#include <memory>
#include <vector>

#include <boost/thread/barrier.hpp>
#include <boost/thread/thread.hpp>
#include <boost/utility.hpp>

#include "dtkPhysMassSpringThread.h"
#include "dtkPhysTetraMassSpring.h"

#include "dtkCollisionDetectPrimitive.h"
#include "dtkIntersectTest.h"

namespace dtk {
class dtkPhysKnotPlanner : public boost::noncopyable {
  // 结

public:
  typedef std::shared_ptr<dtkPhysKnotPlanner> Ptr;

public:
  ~dtkPhysKnotPlanner();
  void KnotRecognition(std::vector<dtkIntersectTest::IntersectResult::Ptr> &);

  void DoKnotFormation();

  // 更新朝向

  void UpdateKnotOrientations();

  void UpdateKnot(double timeslice);

  std::vector<dtkID2> GetKnots() { return mKnots; }

  std::vector<dtkDouble3> GetKnotOrientations() { return mKnotOrientations; }

  std::vector<dtkDouble3> GetKnotCenterPoints() { return mKnotCenterPoints; }

  std::vector<dtkID> GetSegmentInKnots() { return mSegmentInKnots; }

  std::vector<double> GetPointOnSegmentPercent() {
    return mPointOnSegmentPercents;
  }
  void UpdateKnotCenterPoints();

  void UpdateSegmentInKnot();

  static Ptr New(dtkPhysMassSpringThread::Ptr newSutureThread) {
    return Ptr(new dtkPhysKnotPlanner(newSutureThread));
  }

  const std::vector<dtkInterval<int>> &GetAvoidIntervals() {
    return mAvoidIntervals;
  }

public:
  dtkPhysKnotPlanner(dtkPhysMassSpringThread::Ptr newSutureThread);

  bool mDoKnotPlanning;

  std::vector<dtkID2> mKnots; // 结
  dtkID mNumberOfFormatedKnots;
  std::vector<dtkID> mSegmentInKnots;            // 线段数
  std::vector<dtkDouble3> mKnotCenterPoints;     // 中心点
  std::vector<double> mPointOnSegmentPercents;   // 线段细分
  std::vector<dtkDouble3> mKnotOrientations;     // 朝向
  std::vector<bool> mSegmentInKnotOrientations;  // 段的朝向
  std::vector<dtkInterval<int>> mAvoidIntervals; // 间隔

public:
  dtkPhysMassSpringThread::Ptr mSutureThread; // 缝合线
};
}; // namespace dtk

#endif /* SIMPLEPHYSICSENGINE_DTKPHYSKNOTPLANNER_H */
