
/**
 * @file dtkPhysParticle.h
 * @brief  dtkPhysParticle 头文件
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

#ifndef SIMPLEPHYSICSENGINE_DTKPHYSPARTICLE_H
#define SIMPLEPHYSICSENGINE_DTKPHYSPARTICLE_H

#include "dtkGraphicsKernel.h"
#include "dtkIDTypes.h"
#include "dtkTx.h"

namespace dtk {
/**
 * @class <dtkPhysParticle>
 * @brief 粒子
 * @author <>
 * @note
 *
 */
class dtkPhysParticle { // 粒子
public:
  dtkPhysParticle(const GK::Point3 &position, const double &lifetime,
                  const double &mass = 1.0,
                  const dtkT3<double> &vel = dtkT3<double>(0, 0, 0));

public:
  virtual ~dtkPhysParticle();

  /**
   * @brief		更新粒子位置，力,加速度，速度等
   * @param[in]	timeslice : 更新时间间隔
   * @note			更新粒子位置，力,加速度，速度等
   * @return
   *	true update successfully \n
   *	false update failure \n
   */
  bool Update(double timeslice);

  const GK::Point3 &GetPoint() { return mPoint; }
  void SetPoint(const GK::Point3 &point) { mPoint = point; }

  dtkT3<double> GetPosition() {
    return dtkT3<double>(mPoint[0], mPoint[1], mPoint[2]);
  }
  void SetPosition(dtkT3<double> newPos) {
    mPoint = GK::Point3(newPos[0], newPos[1], newPos[2]);
  }

  dtkT3<double> GetVel() { return mVel; }
  void SetVel(dtkT3<double> newVel) { mVel = newVel; }

  void SetMass(const double &mass) { mMass = mass; }
  const double &GetMass() { return mMass; }

  void AddForce(const dtkT3<double> &f) { mForceAccum = mForceAccum + f; }

  const dtkT3<double> &GetForceAccum() { return mForceAccum; }
  void SetForceAccum(dtkT3<double> newForceAccum) {
    mForceAccum = newForceAccum;
  }

  void SetActive(bool newActive) { mActive = newActive; }
  bool IsActive() { return mActive; }

private:
  GK::Point3 mPoint; /**< 点 */

  dtkT3<double> mVel;        /**< 速度 */
  dtkT3<double> mAccel;      /**< 加速度 */
  dtkT3<double> mForceAccum; /**< 合外力 */

  double mMass; /**< 粒子质量 */

  double mResistCoef; /**< 阻力系数 */

  double mLifetime; /**< 粒子生命周期 */

  bool mActive; /**< 是否存活 */
};
} // namespace dtk

#endif /* SIMPLEPHYSICSENGINE_DTKPHYSPARTICLE_H */
