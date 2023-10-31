
/**
 * @file dtkInit.h
 * @brief  dtkInit 头文件
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

#ifndef DTK_INIT_H
#define DTK_INIT_H

namespace dtk
{	

	/**
	 * @brief Whether the dtk has been initialized.
	 *	Defined in dtk.cpp
	 */
	extern bool dtkInited;

	/**
	 * @brief init dtk
	 * @note You must call this function once before you use DTK package.
	 * Defined in dtk.cpp
	*/
    extern "C" void Init();
}

#endif
