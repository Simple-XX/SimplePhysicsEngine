#ifndef VASCULAROPENGLRENDER_H
#define VASCULAROPENGLRENDER_H
#include "dtkPhysTetraMassSpring.h"
#include "dtkStaticTetraMesh.h"
#include "dtkStaticTriangleMesh.h"
#include <cmath>
#include <gl/freeglut.h>

namespace dtk {
class vascularOpenGLRender {
public:
  typedef std::shared_ptr<vascularOpenGLRender> Ptr;
  static Ptr New() { return Ptr(new vascularOpenGLRender()); }

  void SetTriangleMesh(dtkStaticTriangleMesh::Ptr ptr);
  void Draw();
  void Update();
  ~vascularOpenGLRender();

private:
  vascularOpenGLRender();
  dtkStaticTriangleMesh::Ptr mTriangleMeshPtr;
  std::vector<dtkT3<double>> mPoints;
  std::vector<dtkT3<double>> mNormals;
};
} // namespace dtk
#endif