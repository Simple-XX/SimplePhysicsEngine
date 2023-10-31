
/**
 * @file dtkTime.h
 * @brief  dtkTime 头文件
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

#ifndef DTK_TIME_H
#define DTK_TIME_H

#include <ctime>

namespace dtk {
//! Use it to track time
class dtkTime {
  friend dtkTime operator-(const dtkTime &lhs, const dtkTime &rhs);

public:
  dtkTime(const dtkTime &time) : mClicks(time.mClicks) {}

  explicit dtkTime(clock_t time = clock()) : mClicks(time) {}

  inline void SetToCurrentTime() { mClicks = clock(); }

  inline void SetTime(clock_t time) { mClicks = time; }

  inline clock_t GetTime() const { return mClicks; }

  inline float Seconds() const { return ((float)mClicks) / CLOCKS_PER_SEC; }

  inline bool operator==(const dtkTime &rhs) { return mClicks == rhs.mClicks; }

  inline void Advance(double adv) { mClicks += adv * (double)CLOCKS_PER_SEC; }

private:
  clock_t mClicks;
};

inline dtkTime operator-(const dtkTime &lhs, const dtkTime &rhs) {
  return dtkTime(lhs.GetTime() - rhs.GetTime());
}
} // namespace dtk

#endif // DTK_TIME
