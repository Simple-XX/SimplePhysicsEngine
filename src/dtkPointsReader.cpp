
/**
 * @file dtkPointsReader.cpp
 * @brief dtkPointsReader 实现
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

#include <fstream>

#include "dtkAssert.h"
#include "dtkPointsReader.h"

namespace dtk {
void dtkPointsReader::SetOutput(dtkPoints::Ptr pts) {
  dtkAssert(pts.get() != NULL, NULL_POINTER);
  mPts = pts;
}

void dtkPointsReader::SetFileName(const char *fileName) {
  dtkAssert(fileName != NULL, NULL_POINTER);
  mFilePath = std::string(fileName);
}

bool dtkPointsReader::Read() {
  bool rtn = true;

  rtn &= mPts.get() != NULL;
  rtn &= mFilePath.size() > 0;
  if (!rtn) {
    dtkAssert(rtn, ILLEGAL_STATE);
    return false;
  }

  std::ifstream file(mFilePath.c_str());
  size_t numOfPts;
  dtkID maxID;

  file >> numOfPts >> maxID;

  for (size_t i = 0; i < numOfPts; ++i) {
    dtkID id;
    dtkFloat3 coord;

    file >> id >> coord.x >> coord.y >> coord.z;
    mPts->SetPoint(id, GK::Point3(coord.x, coord.y, coord.z));
  }

  file.close();
  return rtn;
}
} // namespace dtk
