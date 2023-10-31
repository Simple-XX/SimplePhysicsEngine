
/**
 * @file dtkPointsWriter.h
 * @brief  dtkPointsWriter 头文件
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

/**
 * @file dtkPointsWriter.h
 * @brief  dtkPointsWriter 头文件
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

#ifndef DTK_POINTSWRITER_H
#define DTK_POINTSWRITER_H

#include <vector>
#include <cstring>
#include <memory>
#include <boost/utility.hpp>

#include "dtkPoints.h"

namespace dtk
{
	class dtkPointsWriter:public boost::noncopyable
	{
	public:
		typedef std::shared_ptr<dtkPointsWriter> Ptr;

		static dtkPointsWriter::Ptr New()
		{
			return dtkPointsWriter::Ptr(new dtkPointsWriter());
		}

	public:

		void SetPoints(dtkPoints::Ptr pts);
		void SetPoints(dtkPoints::Ptr pts, const std::vector<bool> &valid);
		void SetFileName(const char* fileName);

		bool Write();

	private:
		dtkPointsWriter();

		dtkPoints::Ptr	mPts;
		std::string		mFilePath;
		std::vector<bool> mValid;
	};
}

#endif //DTK_POINTSWRITER_H

