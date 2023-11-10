
/**
 * @file dtkImports.h
 * @brief  dtkImports 头文件
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

#ifndef SIMPLEPHYSICSENGINE_DTKIMPORTS_H
#define SIMPLEPHYSICSENGINE_DTKIMPORTS_H

#ifdef DTK_DEBUG
#define DTK_IMPORTS_DEBUG
#endif

#ifdef DTK_IMPORTS_DEBUG
#include <iostream>
#endif

#include "dtkGraphicsKernel.h"
#include "dtkPointsImporter.h"
#include "dtkStaticTetraMeshImporter.h"
#include "dtkStaticTriangleMeshImporter.h"

namespace dtk {
template <class IMPORTER_PTR>
void Import(dtkPoints::Ptr &dst, IMPORTER_PTR &src) {
  dtkID maxID, id;
  size_t numOfPts;
  GK::Point3 pt;

  src->Begin(numOfPts, maxID);
  while (src->Next(id, pt)) {
    dst->SetPoint(id, pt);
  }
  src->End();
}

template <class IMPORTER_PTR>
void Import(dtkStaticTetraMesh::Ptr &dst, IMPORTER_PTR &src) {
#ifdef DTK_IMPORTS_DEBUG
  std::cout << "[Import]" << std::endl;
#endif
  size_t nTetras;
  dtkID4 tetra;

  size_t count = 0;
  src->Begin(nTetras);
#ifdef DTK_IMPORTS_DEBUG
  std::cout << "nTetras: " << nTetras << std::endl;
#endif
  while (src->Next(tetra)) {
    dst->InsertTetra(tetra);
    count++;
  }
  src->End();
#ifdef DTK_IMPORTS_DEBUG
  std::cout << "Number of tetra inserted: " << count << std::endl;
  std::cout << "[/Import]" << std::endl;
  std::cout << std::endl;
#endif
}

template <class IMPORTER_PTR>
void Import(dtkStaticTriangleMesh::Ptr &dst, IMPORTER_PTR &src) {
  size_t nTris;
  dtkID3 tri;

  src->Begin(nTris);
  while (src->Next(tri)) {
    dst->InsertTriangle(tri);
  }
  src->End();
}
} // namespace dtk

#endif /* SIMPLEPHYSICSENGINE_DTKIMPORTS_H */
