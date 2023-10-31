
/**
 * @file dtkTxIO.h
 * @brief dtkTxIO 头文件
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

#ifndef DTK_TXIO_H
#define DTK_TXIO_H

// STL
#include <iostream>

// DTK
#include "dtkConfig.h"
#include "dtkTx.h"

namespace dtk {
template <class T>
inline std::ostream &operator<<(std::ostream &stream, const dtkT2<T> &data) {
  stream << '(' << data.x << ',' << data.y << ')';
  return stream;
}

template <class T>
inline std::ostream &operator<<(std::ostream &stream, const dtkT3<T> &data) {
  stream << '(' << data.x << ',' << data.y << ',' << data.z << ')';
  return stream;
}

template <class T>
inline std::ostream &operator<<(std::ostream &stream, const dtkT4<T> &data) {
  stream << '(' << data.x << ',' << data.y << ',' << data.z << ',' << data.w
         << ')';
  return stream;
}
} // namespace dtk

#endif // DTK_TXIO_H
