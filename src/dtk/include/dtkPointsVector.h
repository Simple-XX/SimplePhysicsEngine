
/**
 * @file dtkPointsVector.h
 * @brief  dtkPointsVector 头文件
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

#ifndef SIMPLEPHYSICSENGINE_DTKPOINTSVECTOR_H
#define SIMPLEPHYSICSENGINE_DTKPOINTSVECTOR_H

#include "dtkConfig.h"

#include "dtkPoints.h"
#include <memory>

namespace dtk {
class dtkPointsVector : public dtkPoints {
public:
  typedef std::shared_ptr<dtkPointsVector> Ptr;

  static dtkPointsVector::Ptr New() {
    return dtkPointsVector::Ptr(new dtkPointsVector());
  }

  static dtkPointsVector::Ptr New(size_t size) {
    return dtkPointsVector::Ptr(new dtkPointsVector(size));
  }

  static dtkPointsVector::Ptr New(const std::vector<GK::Point3> &coords) {
    return dtkPointsVector::Ptr(new dtkPointsVector(coords));
  }

public:
  inline const GK::Point3 &GetPoint(dtkID id) const {
    // dtkAssert(id < mCoords.size(), OUT_OF_RANGE);
    // assert( id < mCoords.size() );
    return mCoords[id];
  }

  inline bool SetPoint(dtkID id, const GK::Point3 &coord) {
    if (id < mCoords.size()) {
      mCoords[id] = coord;
      return true;
    } else {
      // expand the vector
      mCoords.resize(id + 1);
      mCoords[id] = coord;
      return true;
    }
  }

  inline void InsertPoint(dtkID id, const GK::Point3 &coord) {
    dtkAssert(id <= mCoords.size() && id >= 0);
    mCoords.insert(mCoords.begin() + id, coord);
  }

  inline void DeletePoint(dtkID id) {
    dtkAssert(id >= 0 && id < mCoords.size());
    mCoords.erase(mCoords.begin() + id);
  }

  size_t GetNumberOfPoints() const { return mCoords.size(); }

  dtkID GetMaxID() const { return static_cast<dtkID>(mCoords.size() - 1); }

  void Begin() const { mCurPos = 0; }

  bool Next(dtkID &id, GK::Point3 &coord) const {
    bool retVal = false;

    if (mCurPos < mCoords.size()) {
      id = static_cast<dtkID>(mCurPos);
      coord = mCoords[mCurPos++];
      retVal = true;
    }

    return retVal;
  }

private:
  dtkPointsVector() {
    // nothing.
  }

  dtkPointsVector(size_t size) { mCoords.resize(size); }

  dtkPointsVector(const std::vector<GK::Point3> &coords) { mCoords = coords; }

private:
  std::vector<GK::Point3> mCoords;

  // MCurPos's value is modified all the time, even in a const member function.
  mutable size_t mCurPos;
};
} // namespace dtk

#endif /* SIMPLEPHYSICSENGINE_DTKPOINTSVECTOR_H */
