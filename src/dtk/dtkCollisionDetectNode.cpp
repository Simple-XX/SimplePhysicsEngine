
/**
 * @file dtkCollisionDetectNode.cpp
 * @brief dtkCollisionDetectNode 实现
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

#ifdef DTK_DEBUG
#define DTKCOLLISIONDETECTNODE_DEBUG
#endif // DTK_DEBUG
#ifdef DTKCOLLISIONDETECTNODE_DEBUG
#include <iostream>
using namespace std;
#endif

#include "dtkCollisionDetectHierarchy.h"
#include "dtkCollisionDetectNode.h"

namespace dtk {
dtkCollisionDetectNode::dtkCollisionDetectNode(
    dtkCollisionDetectHierarchy *father) {
#ifdef DTKCOLLISIONDETECTNODE_DEBUG
  cout << "[dtkCollisionDetectNode::dtkCollisionDetectNode]" << endl;
  cout << "[/dtkCollisionDetectNode::dtkCollisionDetectNode]" << endl;
  cout << endl;
#endif
  mHierarchy = father;
  mLeaf = true;
  mLevel = 0;

  mMaxLevel = -1;
}

dtkCollisionDetectNode::~dtkCollisionDetectNode() {
#ifdef DTKCOLLISIONDETECTNODE_DEBUG
  cout << "[dtkCollisionDetectNode::~dtkCollisionDetectNode]" << endl;
  cout << "[/dtkCollisionDetectNode::~dtkCollisionDetectNode]" << endl;
  cout << endl;
#endif
  for (dtkID i = 0; i < GetNumOfChildren(); i++) {
    delete mChildren[i];
  }
}

dtkCollisionDetectPrimitive *dtkCollisionDetectNode::GetPrimitive(dtkID id) {
  return mHierarchy->GetPrimitive(mPrimitiveIDs[id]);
}
} // namespace dtk
