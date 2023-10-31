
/**
 * @file dtkStaticTetraMeshWriter.h
 * @brief  dtkStaticTetraMeshWriter 头文件
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

#ifndef DTK_STATICTETRAMESHWRITER_H
#define DTK_STATICTETRAMESHWRITER_H

#include "dtkStaticTetraMesh.h"

#include <cstring>

#include <memory>
#include <boost/utility.hpp>

namespace dtk
{
	class dtkStaticTetraMeshWriter: public boost::noncopyable
	{
	public:

		typedef std::shared_ptr<dtkStaticTetraMeshWriter> Ptr;

		static dtkStaticTetraMeshWriter::Ptr New()
		{
			return dtkStaticTetraMeshWriter::Ptr(new dtkStaticTetraMeshWriter());
		}

	public:
		virtual ~dtkStaticTetraMeshWriter() {}

		void SetInput(dtkStaticTetraMesh::Ptr mesh);
		void SetFileName(const char* filePath);

		bool Write();

	private:
		dtkStaticTetraMeshWriter();

		dtkStaticTetraMesh::Ptr	mMesh;
		std::string			mFilePath;
	};
}

#endif //DTK_STATICTETRAMESHWRITER_H

