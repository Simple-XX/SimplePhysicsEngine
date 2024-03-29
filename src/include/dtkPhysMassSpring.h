
/**
 * @file dtkPhysMassSpring.h
 * @brief  dtkPhysMassSpring 头文件
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

#ifndef SIMPLEPHYSICSENGINE_DTKPHYSMASSSPRING_H
#define SIMPLEPHYSICSENGINE_DTKPHYSMASSSPRING_H

#include <map>
#include <memory>
#include <vector>

#ifdef DTK_CL
#include <CL/cl.h>
#endif

#include "dtkPhysMassPoint.h"
#include "dtkPhysSpring.h"
#include "dtkPoints.h"
#include "dtkPointsVector.h"
#include "dtkStaticTriangleMesh.h"

using namespace std;

#define NWITEMS 256
namespace dtk {
class dtkPhysMassSpring : public boost::noncopyable {
public:
  typedef std::shared_ptr<dtkPhysMassSpring> Ptr;

  static Ptr
  New(double defaultMass = 2, double defaultK = 2000, double defaultDamp = 5880,
      double defaultPointDamp = 1.0, double defaultPointResistence = 2.5,
      dtkDouble3 defaultGravityAccel = dtkDouble3(
          0, 0, 0)) // K  = k0 * cross section area, damp is additional damp
  {
    return Ptr(new dtkPhysMassSpring(defaultMass, defaultK, defaultDamp,
                                     defaultPointDamp, defaultPointResistence,
                                     defaultGravityAccel));
  }

public:
  virtual ~dtkPhysMassSpring();
  dtkPhysMassSpring(double defaultMass, double defaultK, double defaultDamp,
                    double defaultPointDamp,
                    double defaultPointResistence = 2.5,
                    dtkDouble3 defaultGravityAccel = dtkDouble3(0, 0, 0));
  void SetPoints(dtkPoints::Ptr points);
  dtkPoints::Ptr GetPoints();

  const GK::Point3 &GetPoint(dtkID id) const;
  void SetPoint(dtkID id, const GK::Point3 &coord);

  dtkPhysMassPoint *GetMassPoint(dtkID id) { return mMassPoints[id]; };
  dtkID GetNumberOfMassPoints() { return mMassPoints.size(); }
  dtkID AddMassPoint(dtkID id, const double &mass = 1.0,
                     const dtkT3<double> &vel = dtkT3<double>(0, 0, 0),
                     double pointDamp = 1.0, double pointResistence = 2.5,
                     dtkDouble3 defaultGravityAccel = dtkDouble3(0, 0, 0));

  dtkPhysSpring *GetSpring(dtkID id) { return mSprings[id]; };
  dtkID GetNumberOfSprings() { return (dtkID)mSprings.size(); }
  dtkID AddSpring(dtkID p1, dtkID p2, const double &stiff = 0,
                  const double &damp = 0);
  void DeleteSpring(dtkID p1, dtkID p2);

  void SetSpringStiffness(int id, double newK);
  void SetSpringDamp(int id, double newDamp);
  void SetPointMass(int id, double newMass);

  /**
   * @brief		更新弹簧及质点状态
   * @param[in]	timeslice : 更新时间间隔
   * @param[in]	method : 迭代更新算法
   * @param[in]	limitDeformation : 弹簧弹性限度
   * @note			更新弹簧及质点状态
   * @return
   *	true update successfully \n
   *	false update failure \n
   */
  bool Update(double timeslice, ItrMethod method = Euler,
              bool limitDeformation = false);

  /**
   * @brief		单线程更新弹簧及质点状态
   * @param[in]	timeslice : 更新时间间隔
   * @param[in]	method : 迭代更新算法
   * @param[in]	limitDeformation : 弹簧弹性限度
   * @note			单线程更新弹簧及质点状态
   * @return
   *	true update successfully \n
   *	false update failure \n
   */
  bool Update_s(double timeslice, ItrMethod method = Euler,
                bool limitDeformation = false);
  virtual bool PreUpdate(double timeslice, ItrMethod method = Euler,
                         dtkID iteration = 0);
  virtual bool PostUpdate(ItrMethod method = Euler, dtkID iteration = 0);

  // 更新迭代
  bool Update_iteration(double timeslice, ItrMethod method = Euler,
                        dtkID iteration = 0, bool limitDeformation = false);

  // 应用冲量
  bool ApplyImpulse(double timeslice);

  /**
   * @brief		更新弹簧状态，点位置，速度等
   * @param[in]	timeslice : 更新时间间隔
   * @param[in]	method : 迭代更新算法
   * @param[in]	limitDeformation : 弹簧弹性限度
   * @note			更新弹簧状态，点位置，速度等
   * @return
   *	true update successfully \n
   *	false update failure \n
   */

  bool UpdateStrings(double timeslice, ItrMethod method = Euler,
                     dtkID iteration = 0, bool limitDeformation = false);

  /**
   * @brief		更新质点状态，点位置，速度等
   * @param[in]	timeslice : 更新时间间隔
   * @param[in]	method : 迭代更新算法
   * @param[in]	limitDeformation : 弹簧弹性限度
   * @note			更新质点状态，点位置，速度等
   * @return
   *	true update successfully \n
   *	false update failure \n
   */

  bool UpdateMassPoints(double timeslice, ItrMethod method = Euler,
                        dtkID iteration = 0);

  // 从三角网格添加质量弹簧

  void SetTriangleMesh(dtkStaticTriangleMesh::Ptr newTriangleMesh);

  // 寻找周围距离小于distance的twins点.

  size_t FindTwins(Ptr, double distance);
  void AbandonTwins();

  void RegisterLabel(dtkID label);

  void TransportForce(double timeslice);

  const dtkT3<double> &GetTransportForce(dtkID label) {
    return mTransportForces[label];
  }

  void ConvertImpulseToForce(double timeslice);

  const dtkT3<double> &GetImpulseForce() { return mImpulseForce; }

  bool IsUnderControl() { return mUnderControl; }

  void SetUnderControl(bool underControl) { mUnderControl = underControl; }

  dtkPhysSpring *GetSpringByPoints(dtkID2);

#ifdef DTK_CL
  /**
   * OpenCL related initialisations.
   * Set up Context, Device list, Command Queue, Memory buffers
   * Build CL kernel program executable
   * @return 1 on success and 0 on failure
   */
  bool Update_mt(double timeslice, ItrMethod method = Euler,
                 bool limitDeformation = false);
  int SetupCL();
  int RunCLKernels(double timeslice, dtkID iteration);
  bool Open(const char *fileName);
  void InitialCLArray();
  void UpdateCopyToCLArray();
  void UpdateCopyBack();
  void CheckCLStatus(cl_int, std::string);
  int WaitForEventAndRelease(cl_event *);
  void UseMultiThread(const char *fileName);
#endif

#ifdef DTK_DEBUG
public:
  std::vector<dtkT3<double>> mAltitudeSpringForces;
  std::vector<dtkT3<double>> mTotalSpringForces;
#endif

protected:
  dtkPoints::Ptr mPts;                         /**< 点集 */
  std::vector<dtkPhysMassPoint *> mMassPoints; /**< 质点集 */
  std::vector<dtkPhysSpring *> mSprings;       /**< 弹簧 */

  // 边集
  std::map<dtkID2, dtkPhysSpring *>
      mEdgeMap; /**< map from spring specified by dtkID2 to spring id */

  double mDefaultMass;             /**< 质量 */
  double mDefaultStiff;            /**< 弹簧刚性系数，弹性系数 */
  double mDefaultDamp;             /**< 弹簧阻尼 */
  double mDefaultPointDamp;        /**< 点阻尼 */
  double mDefaultPointResistence;  /**< 阻力 */
  dtkDouble3 mDefaultGravityAccel; /**< 重力加速度 */

  bool mUnderControl; /**< 弹簧受控 */

  std::vector<dtkID> mLabels; // 标记点集, 可给予Transport力

  std::map<dtkID, dtkT3<double>> mTransportForces; //
  dtkT3<double> mImpulseForce;                     // 瞬时力

#ifdef DTK_CL
  bool mUseMultiThread;
  // OpenCL
  cl_float mTimesliceData;
  cl_int mIterationData;
  cl_int mNumberOfMassPoints;

  cl_int *mSpringVertexIDData;
  cl_mem mSpringVertexIDBuffer;
  cl_float *mSpringOriLengthData;
  cl_mem mSpringOriLengthBuffer;
  cl_float *mSpringStiffData;
  cl_mem mSpringStiffBuffer;
  cl_float *mSpringDampData;
  cl_mem mSpringDampBuffer;

  cl_float *mMPPosData;
  cl_mem mMPPosBuffer;
  cl_float *mMPVelData;
  cl_mem mMPVelBuffer;
  cl_float *mMPAccelData;
  cl_mem mMPAccelBuffer;
  cl_float *mMPForceAccumData;
  cl_mem mMPForceAccumBuffer;
  cl_float *mMPGravityData;
  cl_mem mMPGravityBuffer;
  cl_float *mMPMassData;
  cl_mem mMPMassBuffer;
  cl_float *mMPPosBufferData;
  cl_mem mMPPosBufferBuffer;
  cl_float *mMPVelBufferData;
  cl_mem mMPVelBufferBuffer;
  cl_float *mMPAccelBufferData;
  cl_mem mMPAccelBufferBuffer;
  cl_float *mMPForceDecoratorData;
  cl_mem mMPForceDecoratorBuffer;
  cl_bool *mMPActiveData;
  cl_mem mMPActiveBuffer;

  // new spring calculate
  cl_int *mMPAdjacentVertexIDsData;
  cl_mem mMPAdjacentVertexIDsBuffer;
  cl_int *mMPNumberOfAdjacentVertexData;
  cl_mem mMPNumberOfAdjacentVertexBuffer;
  cl_int *mMPStartAdjacentVertexIDData;
  cl_mem mMPStartAdjacentVertexIDBuffer;

  cl_int *mMPAdjacentSpringIDsData;
  cl_mem mMPAdjacentSpringIDsBuffer;

  size_t maxWorkGroupSize;
  size_t *maxWorkItemSizes;

  cl_context context;
  cl_platform_id platform;
  cl_device_id device;

  cl_command_queue queue;
  cl_program program;
  cl_kernel kernelSpring;
  cl_kernel kernelMP;
  size_t global_work_size;
  size_t work_size;
  std::string source_;
#endif
};
}; // namespace dtk

#endif /* SIMPLEPHYSICSENGINE_DTKPHYSMASSSPRING_H */
