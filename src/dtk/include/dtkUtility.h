
/**
 * @file dtkUtility.h
 * @brief  dtkUtility 头文件
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

#ifndef DTK_UTILITY_H
#define DTK_UTILITY_H

namespace dtk
{
	//return true if low <= val <= up
	//you must override operator< for T.
	template <class T>
	inline bool Between(const T &val, const T &low, const T &up)
	{
		return (!(val < low)) && (!(up < val));
	}

	template <class T>
	inline void CheckBetween(const T &val, const T &low, const T &up)
	{
		dtkAssert(Between(val, low, up), OUT_OF_RANGE);
	}

	//Clamp the val into [a, b].
	//When val < a, return a; val > b, return b
	//Otherwise, return val.
	template <class T>
	inline T dtkClamp(T val, const T &a, const T &b)
	{
		if (val < a) return a;
		if (b < val) return b;
		return val;
	}

    template <class T>
    inline void Sort(T &a, T &b, T &c)
    {
        if (b < a) std::swap(a, b);
        if (c < a) std::swap(a, c);
        if (c < b) std::swap(b, c);
    }
}
#endif //DTK_UTILITY_H
