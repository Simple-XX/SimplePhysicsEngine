
/**
 * @file dtkPhysSpring.h
 * @brief  dtkPhysSpring 头文件
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

#ifndef SIMPLEPHYSICSENGINE_DTKPHYSSPRING_H
#define SIMPLEPHYSICSENGINE_DTKPHYSSPRING_H

#include "dtkIDTypes.h"
#include "dtkPhysMassPoint.h"

namespace dtk {
  class dtkPhysSpring {
  public:
    dtkPhysSpring(dtkPhysMassPoint* p1, dtkPhysMassPoint* p2,
                  const double& stiff = 5, const double& damp = 0);

    dtkPhysMassPoint* GetFirstVertex() { return mVerteces[0]; }
    dtkPhysMassPoint* GetSecondVertex() { return mVerteces[1]; }

    // 更新力到质点,迭代更新
    bool Update(double timeslice, ItrMethod method = Euler, dtkID iteration = 0,
                bool limitDeformation = false);
    void SetStiffness(double newStiffness) { mStiffness = newStiffness; }
    void SetDamp(double newDamp) { mDamp = newDamp; }
    double GetOriLength() { return mOriLength; }
    void SetRestLength(const double& newRestLength) { mRestLength = newRestLength; }
    double GetRestLength() { return mRestLength; }
    double GetStiffness() { return mStiffness; }
    double GetDamp() { return mDamp; }

  public:
    virtual ~dtkPhysSpring();

  private:
    dtkPhysMassPoint* mVerteces[2]; /**< 弹簧两个质点 */

    double mOriLength; /**< 原始长度 */
    double mRestLength; /**< 静息长度 */
    double mStiffness; /**< 刚度 */
    double mDamp;      /**<  阻尼 */
  };
} // namespace dtk

#endif /* SIMPLEPHYSICSENGINE_DTKPHYSSPRING_H */
