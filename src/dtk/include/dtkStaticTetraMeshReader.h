
/**
 * @file dtkStaticTetraMeshReader.h
 * @brief  dtkStaticTetraMeshReader 头文件
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

#ifndef SIMPLEPHYSICSENGINE_DTKSTATICTETRAMESHREADER_H
#define SIMPLEPHYSICSENGINE_DTKSTATICTETRAMESHREADER_H

#include <cstring>
#include <memory>

#include <boost/utility.hpp>

#include "dtkStaticTetraMesh.h"

namespace dtk {
class dtkStaticTetraMeshReader : public boost::noncopyable {
public:
  typedef std::shared_ptr<dtkStaticTetraMeshReader> Ptr;

  static dtkStaticTetraMeshReader::Ptr New() {
    return dtkStaticTetraMeshReader::Ptr(new dtkStaticTetraMeshReader());
  }

public:
  virtual ~dtkStaticTetraMeshReader() {}

  void SetOutput(dtkStaticTetraMesh::Ptr mesh);
  void SetFileName(const char *filePath);

  bool Read();

private:
  dtkStaticTetraMeshReader();

  dtkStaticTetraMesh::Ptr mMesh;
  std::string mFilePath;
};
} // namespace dtk

#endif /* SIMPLEPHYSICSENGINE_DTKSTATICTETRAMESHREADER_H */
