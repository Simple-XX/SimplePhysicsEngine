
/**
 * @file dtkPoints.h
 * @brief  dtkPoints 头文件
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

#ifndef SIMPLEPHYSICSENGINE_DTKPOINTS_H
#define SIMPLEPHYSICSENGINE_DTKPOINTS_H

#include <memory>

#include <boost/utility.hpp>

#include "dtkConfig.h"
#include "dtkGraphicsKernel.h"
#include "dtkIDTypes.h"
#include "dtkTx.h"

namespace dtk {
//! A points container
/*! It is used in many graphical element in DTK,
 *  such as Mesh, Graph
 */
class dtkPoints : public boost::noncopyable {
public:
  typedef std::shared_ptr<dtkPoints> Ptr;

public:
  virtual ~dtkPoints() {}
  virtual const GK::Point3 &GetPoint(dtkID id) const = 0;
  virtual bool SetPoint(dtkID id, const GK::Point3 &coord) = 0;

  virtual size_t GetNumberOfPoints() const = 0;
  virtual dtkID GetMaxID() const = 0;

  virtual void Begin() const = 0;
  virtual bool Next(dtkID &id, GK::Point3 &coord) const = 0;

  virtual void InsertPoint(dtkID id, const GK::Point3 &coord) = 0;
  virtual void DeletePoint(dtkID id) = 0;
};
} // namespace dtk

#include "dtkPointsVector.h"

#endif /* SIMPLEPHYSICSENGINE_DTKPOINTS_H */
