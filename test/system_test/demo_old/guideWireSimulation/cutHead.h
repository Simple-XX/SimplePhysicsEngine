#ifndef CUTHEAD_H
#define CUTHEAD_H

#include "dtkStaticTriangleMesh.h"
#include "dtkTx.h"
#include <vector>

namespace dtk {
class cutHead {
public:
  cutHead(dtkStaticTriangleMesh::Ptr triangleMesh,
          const std::vector<dtkID> &avoidPoint);
  const std::vector<dtkID3> &AvoidTriangle();

private:
  dtkStaticTriangleMesh::Ptr mTriangleMesh;
  std::vector<dtkID> mAvoidPoint;
  std::vector<dtkID3> mAvoidTriangle;
};
} // namespace dtk

#endif