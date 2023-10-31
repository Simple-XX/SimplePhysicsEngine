#ifndef GUIDEWIRE_H
#define GUIDEWIRE_H

// dtk�е�ͷ�ļ�
#include "dtkPhysMassPoint.h"

// std�е�ͷ�ļ�
#include <vector>
using namespace std;

namespace dtk {
class guideWire : public boost::noncopyable {
  //  ���г�Ա����
public:
  typedef std::shared_ptr<guideWire> Ptr;
  static Ptr New() { return Ptr(new guideWire()); }

  // ��˿�Ķ�̬����
  void Update(double timeslice, ItrMethod method = Euler, dtkID iteration = 0);
  void UpdateMassPoints(double timeslice, ItrMethod method = Euler,
                        dtkID iteration = 0);
  void PreUpdate(double timeslice);

  // ��˿�Ĳ�������
  void SetPoints(dtkPoints::Ptr points);
  void SetSegInterval(double segInterval);
  void SetTipSegInterval(double tipSegInterval);
  void SetLastTipID(dtkID lastTipID);
  void SetBendModulus(double bendModulus);
  void SetTipBendModulus(double tipBendModulus);
  void Set3DBendModulus(double threeDBendModulus);
  void SetTipOriginAngle(const vector<double> &tipOriginAngle);
  void SetMass(double mass);
  void SetPointResistence(double pointResistence);
  void SetTipPointResistence(double tipPointResistence);
  void SetContactForces(dtkID id, const dtkDouble3 &force);
  void SetCollisionFlag(dtkID id, bool flag);
  void ResetContactForces();
  void ResetCollisionFlag();
  void SetGuideWireStartPoint(const GK::Point3 &point);
  void SetGuideWireStartDirection(const GK::Vector3 &direction);

  // ��ȡ��˿�Ĳ���
  dtkPoints::Ptr GetPoints() const;
  double GetSegInterval() const;
  double GetTipSegInterval() const;
  dtkID GetLastTipID() const;
  double GetBendModulus() const;
  double Get3DBendModulus() const;
  vector<double> GetTipOriginAngle() const;
  double GetMass() const;
  double GetPointResistence() const;
  double GetTipPointResistence() const;
  double GetTipBendModulus() const;

  // ��˿�Ĺ��ܺ���
  void RemovePoint(); // �Ƴ���˿�岿����һ���ʵ������
  void AddPoint(const GK::Point3 &point); // ����˿�岿�ĺ�������һ���ʵ������
  void DynamicGuideWirePoint(); // ���ݵ�˿���˶�����ɾ�ʵ�

  // ��˿��ģ�ͺ���
  void ConstructGuideWireMassPoints(); // ����˿��������ɢ���ʵ�ģ��
  dtkPhysMassPoint *GetMassPoint(dtkID id) const;
  dtkID AddMassPoint(const dtkT3<double> &vel);
  bool RemoveMassPoint();               // �Ƴ�һ����˿�ʵ�
  void ResistStretch(double timeslice); // ��˿�Ŀ�����
  void AddBendForce(double timeslice); // ��˿�ܵ���������
  void ResistOverBend(double timeslice); // ��λ�÷������Ƶ�˿��˵�����
  void Add3DBendForce(double timeslice); // ��˿�ܵ���ƽ��������
  void AddTwistForce(double twistAngle); // ��˿�ܵ���Ť����
  void AddContactForce(); // ��˿���˶��Ĺ����ܵ�Ѫ�ܱڵĽӴ���

  // �Ե�˿��������λ�ú��ٶȽ��в���
  void AddForce(const dtkT3<double> &force, dtkID directionID,
                bool isPush); // �û����������Ĵ�������
  // �û��Ե�˿����������
  void ApplyExternalForce(const dtkT3<double> &force); //	�û����ơ�������
  void RotateMassPoint(dtkPoints::Ptr points, dtkID rotatePointID,
                       dtkID axisPointID1, dtkID axisPointID2, double angle);

  // ��̬��������ת������
  static dtkT3<double> vector3TodtkT3(const GK::Vector3 &v3) {
    return dtkT3<double>(v3.x(), v3.y(), v3.z());
  }

public:
  vector<dtkDouble3> mContactForces; // Ѫ�ܱڶԵ�˿�ĽӴ���

  // ˽�г�Ա����
private:
  guideWire();

  // ˽�����ݳ�Ա
  double mSegInterval;    // ��˿��ԭʼ�γ�
  double mTipSegInterval; // ��˿��˵Ķγ�
  dtkPoints::Ptr mPts;    // ��˿���ʵ�����
  dtkID mLastTipID;       // ��˿�����ĩ�ε�ID
  double mBendModulus;    // ��˿�岿�ֵ�����ģ��
  double mTipBendModulus; // ��˿��˲��ֵĶ�ά����ģ��
  double m3DBendModulus;  // ��˿��˲��ֵ�3D����ģ��
  vector<double> mTipOriginAngle; // ��˿��˲��ֵ�ԭʼ�Ƕ�
  double mMass;                   // ��˿������
  double mTimeslice;              //  ������ʱ��Ƭ��С
  double mDefaultPointResistence; // ��˿�岿��Ѫ���е�����ϵ��
  double mDefaultTipPointResistence; // ��˿�����Ѫ���е�����ϵ��
  double mDefaultPointDamp; // ��˿��Ѫ�����ٶȵĽ��ͱ���
  double mTwistSlice; // ��˿��Ѫ���е�Ť���Ƕ�Ƭ��С
  double mDefaultGravityAccel;          // Ĭ�ϵ��������ٶ�
  double mIncompleteSegLength;          // �������ĵ�˿�γ�
  GK::Point3 mGuideWireStartPoint;      // ��˿����ʼλ��
  GK::Vector3 mGuideWireStartDirection; // ��˿����ʼ����
  vector<dtkPhysMassPoint *> mMassPoints; // ��˿���ʵ�
  vector<bool>
      mMassPointsCollisionFlag; // �ʵ���Ѫ�ܱڷ�����ײ��Flag�����з�����ײΪtrue������Ϊfalse
};
} // namespace dtk
#endif