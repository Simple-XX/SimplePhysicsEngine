
/**
 * @file dtkStaticTriangleMeshReader.h
 * @brief  dtkStaticTriangleMeshReader 头文件
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

#ifndef SIMPLEPHYSICSENGINE_DTKSTATICTRIANGLEMESHREADER_H
#define SIMPLEPHYSICSENGINE_DTKSTATICTRIANGLEMESHREADER_H

#include "dtkStaticTriangleMesh.h"

#include <boost/utility.hpp>
#include <memory>

namespace dtk {
class dtkStaticTriangleMeshReader : public boost::noncopyable {
public:
  typedef std::shared_ptr<dtkStaticTriangleMeshReader> Ptr;

  static dtkStaticTriangleMeshReader::Ptr New() {
    return dtkStaticTriangleMeshReader::Ptr(new dtkStaticTriangleMeshReader());
  }

public:
  virtual ~dtkStaticTriangleMeshReader() {}

  void SetOutput(dtkStaticTriangleMesh::Ptr mesh);
  void SetFileName(const char *filePath);

  bool Read();

private:
  dtkStaticTriangleMeshReader();

  dtkStaticTriangleMesh::Ptr mMesh;
  std::string mFilePath;
};
} // namespace dtk

#endif /* SIMPLEPHYSICSENGINE_DTKSTATICTRIANGLEMESHREADER_H */
