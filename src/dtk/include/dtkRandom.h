
/**
 * @file dtkRandom.h
 * @brief  dtkRandom 头文件
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

#ifndef DTK_RANDOM_H
#define DTK_RANDOM_H

namespace dtk {
// Get a random int x which between a <= x < b
inline int dtkRandomInt(int a, int b) {
  int r = std::rand() % (b - a);
  return a + r;
}

// Generate a random float in [0.0, 1.0]
inline float dtkRandomFloat() { return ((float)rand()) / ((float)RAND_MAX); }

// Generate a random double in [0.0, 1.0]
inline double dtkRandomDouble() {
  return ((double)rand()) / ((double)RAND_MAX);
}
} // namespace dtk

#endif // DTK_RANDOM_H
