
/**
 * @file dtkAssert.h
 * @brief  dtkAssert 头文件
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

#ifndef SIMPLEPHYSICSENGINE_DTKASSERT_H
#define SIMPLEPHYSICSENGINE_DTKASSERT_H

#include "dtkConfig.h"
#include "dtkError.h"
#include "dtkErrorManager.h"

namespace dtk {
#ifdef DTK_DEBUG
inline bool dtkAssert(bool cond, dtkError err = UNKNOW_ERROR) {
  if (!cond)
    dtkErrMgr.PushError(err);
  if (cond)
    return true;

  assert(cond);
  return cond;
}
#else

// #define dtkAssert( cond, err ) ( (void) 0 )
inline bool dtkAssert(bool cond, dtkError err = UNKNOW_ERROR) {
  if (!cond)
    dtkErrMgr.PushError(err);
  return cond;
}

#endif // DTK_DEBUG

} // namespace dtk

#endif /* SIMPLEPHYSICSENGINE_DTKASSERT_H */
