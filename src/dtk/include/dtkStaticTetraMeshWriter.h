
/**
 * @file dtkStaticTetraMeshWriter.h
 * @brief  dtkStaticTetraMeshWriter 头文件
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

#ifndef SIMPLEPHYSICSENGINE_DTKSTATICTETRAMESHWRITER_H
#define SIMPLEPHYSICSENGINE_DTKSTATICTETRAMESHWRITER_H

#include "dtkStaticTetraMesh.h"

#include <cstring>

#include <boost/utility.hpp>
#include <memory>

namespace dtk {
class dtkStaticTetraMeshWriter : public boost::noncopyable {
public:
  typedef std::shared_ptr<dtkStaticTetraMeshWriter> Ptr;

  static dtkStaticTetraMeshWriter::Ptr New() {
    return dtkStaticTetraMeshWriter::Ptr(new dtkStaticTetraMeshWriter());
  }

public:
  virtual ~dtkStaticTetraMeshWriter() {}

  void SetInput(dtkStaticTetraMesh::Ptr mesh);
  void SetFileName(const char *filePath);

  bool Write();

private:
  dtkStaticTetraMeshWriter();

  dtkStaticTetraMesh::Ptr mMesh;
  std::string mFilePath;
};
} // namespace dtk

#endif /* SIMPLEPHYSICSENGINE_DTKSTATICTETRAMESHWRITER_H */
