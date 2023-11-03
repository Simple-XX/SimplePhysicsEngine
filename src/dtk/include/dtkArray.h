
/**
 * @file dtkArray.h
 * @brief  dtkArray 头文件
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

#ifndef SIMPLEPHYSICSENGINE_DTKARRAY_H
#define SIMPLEPHYSICSENGINE_DTKARRAY_H

#include <memory>
#include <vector>

#include <boost/utility.hpp>

namespace dtk {
template <class T> class dtkArray : boost::noncopyable {
public:
  typedef std::shared_ptr<dtkArray<T>> Ptr;

  static typename dtkArray::Ptr New(size_t size = 100) {
    return typename dtkArray<T>::Ptr(new dtkArray<T>(size));
  }

public:
  const T &At(size_t offset) const {
    dtkAssert(offset < mData.size(), OUT_OF_RANGE);
    return mData[offset];
  }

  T &At(size_t offset) {
    dtkAssert(offset < mData.size(), OUT_OF_RANGE);
    return mData[offset];
  }

  T GetAt(size_t offset) const {
    dtkAssert(offset < mData.size(), OUT_OF_RANGE);
    return mData[offset];
  }

  void SetAt(size_t offset, const T &data) {
    if (offset >= mData.size())
      mData.resize(offset * 2);

    mData[offset] = data;
  }

  void PushBack(const T &data) { mData.push_back(data); }

  void Reserve(size_t size) { mData.reserve(size); }

  size_t Size() const { return mData.size(); }

private:
  dtkArray(size_t size) { mData.reserve(size); }

  std::vector<T> mData;
};
} // namespace dtk

#endif /* SIMPLEPHYSICSENGINE_DTKARRAY_H */
