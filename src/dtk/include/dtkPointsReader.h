
/**
 * @file dtkPointsReader.h
 * @brief  dtkPointsReader 头文件
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

#ifndef DTK_POINTSREADER_H
#define DTK_POINTSREADER_H

#include "dtkPoints.h"

#include <boost/utility.hpp>
#include <memory>

namespace dtk {
class dtkPointsReader : public boost::noncopyable {
public:
  typedef std::shared_ptr<dtkPointsReader> Ptr;

  static dtkPointsReader::Ptr New() {
    return dtkPointsReader::Ptr(new dtkPointsReader());
  }

public:
  void SetFileName(const char *filePath);
  void SetOutput(dtkPoints::Ptr pts);

  bool Read();

private:
  dtkPoints::Ptr mPts;
  std::string mFilePath;
};
} // namespace dtk

#endif // DTK_POINTSREADER_H
