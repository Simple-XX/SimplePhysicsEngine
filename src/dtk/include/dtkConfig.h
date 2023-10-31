
/**
 * @file dtkConfig.h
 * @brief  dtkConfig 头文件
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

#ifndef DTK_CONFIG_H
#define DTK_CONFIG_H

#ifndef NDEBUG
//    #define DTK_DEBUG
#endif //DEBUG

#define DTK_WINDOWS

#define DTK_BLAS_MKL

//Comment this line if you don't want to use CUDA.
//#define DTK_CUDA	

#define DTK_CGAL

//#define DTK_TBB

//#define DTK_CL

//Comment this line if you don't want to use CUDA.
#define DTK_GLM

#include <limits>

namespace dtk
{
	//Constants
	#define dtkFloatMax ((std::numeric_limits<float>::max)())
	#define dtkFloatMin (-dtkFloatMax)

	#define dtkDoubleMax ((std::numeric_limits<double>::max)())
	#define dtkDoubleMin (-dtkDoubleMax)

    #define dtkIntMax ((std::numeric_limits<int>::max)())
    #define dtkIntMin (-dtkIntMax)

	const float dtkFloatEpslon = (std::numeric_limits<float>::min)() * 10.0f;
	const float dtkDoubleEpslon = (std::numeric_limits<float>::min)() * 10.0f;
	const float dtkFloatError  = dtkFloatMax;
	const float dtkPI = 3.14159265357429f;

	typedef unsigned int 		dtkWORD;
	typedef unsigned long long 	dtkDWORD;

	const auto inf = std::numeric_limits<double>::infinity();
};

#endif //DTK_CONFIG_H
