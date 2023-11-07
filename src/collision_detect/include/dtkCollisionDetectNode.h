
/**
 * @file dtkCollisionDetectNode.h
 * @brief  dtkCollisionDetectNode 头文件
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

#ifndef SIMPLEPHYSICSENGINE_DTKCOLLISIONDETECTNODE_H
#define SIMPLEPHYSICSENGINE_DTKCOLLISIONDETECTNODE_H

#include <memory>
#include <vector>

#include <boost/utility.hpp>

#include "dtkCollisionDetectPrimitive.h"
#include "dtkConfig.h"
#include "dtkIDTypes.h"

namespace dtk {
class dtkCollisionDetectHierarchy;

/**
 * @class <dtkCollisionDetectNode>
 * @brief 冲突检测树结点
 * @author <>
 * @note
 * 冲突检测树结点类，包含一个dtkCollisionDetectHierarchy针，指向一组图元。
 */
class dtkCollisionDetectNode {
public:
  dtkCollisionDetectNode(dtkCollisionDetectHierarchy *father);

  virtual ~dtkCollisionDetectNode();

  virtual void Split() = 0;

  virtual void Update() = 0;

  inline bool IsLeaf() const { return mLeaf; }

  inline void SetLeaf(bool leaf) { mLeaf = leaf; }

  inline void AddPrimitive(dtkID id) { mPrimitiveIDs.push_back(id); }

  inline void AddPrimitive(dtkCollisionDetectPrimitive *primitive) {
    mPrimitiveIDs.push_back(primitive->mLocalID);
  }

  inline void DeletePrimitive(dtkCollisionDetectPrimitive *primitive) {
    primitive->mActive = false;
    std::vector<dtkID>::iterator it;
    it = std::find(mPrimitiveIDs.begin(), mPrimitiveIDs.end(),
                   primitive->mLocalID);
    assert(it != mPrimitiveIDs.end());
    mPrimitiveIDs.erase(it);
  }

  inline size_t GetNumOfPrimitives() const { return mPrimitiveIDs.size(); }

  dtkCollisionDetectPrimitive *GetPrimitive(dtkID id);

  inline size_t GetNumOfChildren() const { return mChildren.size(); }

  inline size_t GetLevel() const { return mLevel; }

  inline void SetMaxLevel(size_t level) { mMaxLevel = level; }

  inline dtkCollisionDetectNode *GetChild(size_t id) { return mChildren[id]; }

protected:
  dtkCollisionDetectHierarchy
      *mHierarchy; /**< 冲突检测树一个层，包含一组图元 */
  std::vector<dtkID> mPrimitiveIDs;                /**< 图元ID的集合 */
  std::vector<dtkCollisionDetectNode *> mChildren; /**< 冲突检测树的子节点 */
  bool mLeaf;    /**< 当前节点是否为叶节点 */
  size_t mLevel; /**< 当前节点所处层数 */

  size_t mMaxLevel; /**< 最大层数 */
};
} // namespace dtk

// #include "impl/dtkCollisionDetectNodeImpl.hpp"

#endif /* SIMPLEPHYSICSENGINE_DTKCOLLISIONDETECTNODE_H */
