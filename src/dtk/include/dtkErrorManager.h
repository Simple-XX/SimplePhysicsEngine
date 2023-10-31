
/**
 * @file dtkErrorManager.h
 * @brief  dtkErrorManager 头文件
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

#ifndef DTK_ERRORMANAGER_H
#define DTK_ERRORMANAGER_H

// STL headers
#include <map>
#include <stack>
#include <string>
#include <vector>

#include <iostream>

#include <boost/utility.hpp>

// DTK headers
#include "dtkConfig.h"
#include "dtkError.h"

namespace dtk {
//! A standard error reporter
/*!
 * Implemented in Singleton
 */
class dtkErrorManager : public boost::noncopyable {
public:
  static dtkErrorManager &GetInstance();

  size_t GetNumberOfErrors();

  dtkError GetLatestError();
  const std::string &GetErrorString(dtkError error);

  void PushError(dtkError error);
  void PopError(dtkError error);

  void Reset();

private:
  dtkErrorManager();
  static dtkErrorManager msErrorMgr;

  void InitErrorStrings();
  void InitErrorString(dtkError id, const char *str);

private:
  std::map<dtkError, std::string> mErrStrs;
  std::stack<dtkError> mErrors;
};

extern dtkErrorManager &dtkErrMgr;
} // namespace dtk

#endif // DTK_ERRORMANAGER_H
