
/**
 * @file dtkExports.h
 * @brief  dtkExports 头文件
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

#ifndef SIMPLEPHYSICSENGINE_DTKEXPORTS_H
#define SIMPLEPHYSICSENGINE_DTKEXPORTS_H

#include <vector>

#include "dtkPoints.h"
#include "dtkStaticTetraMesh.h"
#include "dtkStaticTriangleMesh.h"

namespace dtk {
template <class EXPORTER_PTR>
inline void Export(const EXPORTER_PTR &dst, dtkPoints::Ptr src) {
  dtkID id;
  GK::Point3 coord;

  size_t numOfPts = src->GetNumberOfPoints();
  dtkID maxID = src->GetMaxID();

  dst->Begin(numOfPts, maxID);
  src->Begin();
  while (src->Next(id, coord)) {
    dst->SetPoint(id, coord);
  }
  dst->End();
}

template <class EXPORTER_PTR>
inline void Export(const EXPORTER_PTR &dst, dtkStaticTetraMesh::Ptr src) {
  dtkID4 tetra;

  const std::vector<dtkID4> &ec = src->GetECTable();
  size_t n = ec.size();

  dst->Begin(n);
  for (size_t i = 0; i < n; ++i) {
    dst->InsertTetra(ec[i]);
  }
  dst->End();
}

template <class EXPORTER_PTR>
inline void Export(const EXPORTER_PTR &dst, dtkStaticTriangleMesh::Ptr src) {
  dtkID3 tri;

  const std::vector<dtkID3> &ec = src->GetECTable();
  size_t n = ec.size();

  dst->Begin(n);
  for (size_t i = 0; i < n; ++i) {
    dst->InsertTriangle(ec[i]);
  }
  dst->End();
}
} // namespace dtk

#endif /* SIMPLEPHYSICSENGINE_DTKEXPORTS_H */
