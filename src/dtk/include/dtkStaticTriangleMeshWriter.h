
/**
 * @file dtkStaticTriangleMeshWriter.h
 * @brief  dtkStaticTriangleMeshWriter 头文件
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

#ifndef SIMPLEPHYSICSENGINE_DTKSTATICTRIANGLEMESHWRITER_H
#define SIMPLEPHYSICSENGINE_DTKSTATICTRIANGLEMESHWRITER_H

#include "dtkStaticTriangleMesh.h"

#include <cstring>

#include <boost/utility.hpp>
#include <memory>

namespace dtk {
class dtkStaticTriangleMeshWriter : boost::noncopyable {
public:
  typedef std::shared_ptr<dtkStaticTriangleMeshWriter> Ptr;

  static dtkStaticTriangleMeshWriter::Ptr New() {
    return dtkStaticTriangleMeshWriter::Ptr(new dtkStaticTriangleMeshWriter());
  }

public:
  virtual ~dtkStaticTriangleMeshWriter() {}

  void SetInput(dtkStaticTriangleMesh::Ptr mesh);
  void SetFileName(const char *filePath);

  bool Write();

private:
  dtkStaticTriangleMeshWriter();

  dtkStaticTriangleMesh::Ptr mMesh;
  std::string mFilePath;
};
} // namespace dtk

#endif /* SIMPLEPHYSICSENGINE_DTKSTATICTRIANGLEMESHWRITER_H */
