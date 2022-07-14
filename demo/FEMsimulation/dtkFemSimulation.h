/*
 * @Author: tom: https://github.com/TOMsworkspace
 * @Date: 2021-09-03 16:12:05
 * @Last Modified by: tom: https://github.com/TOMsworkspace
 * @Last Modified time: 2021-09-03 17:22:45
 */


#ifndef DTKFEMSIMULATION_H
#define DTKFEMSIMULATION_H

#include "dtkScene.h"
#include "dtkMesh.h"
#include "../../src/dtk.h"
#include "../../src/dtkGraphicsKernel.h"
#include <iostream>
#include <vector>

#include <Eigen/Dense>
using namespace std;
using namespace Eigen;
using namespace dtk;

class dtkFemSimulation : public dtkScene
{

private:
	dtk::dtkGraphicsKernel::Circle2 sphere;//(dtk::dtkGraphicsKernel::Point2(0.5,0.2), 0.1);
	Vector2f spherecenter;

public:
	dtkMesh rectangle;
	dtkFemSimulation(unsigned int width, unsigned int height);
	~dtkFemSimulation();
	void Init();

	void moveBall(int x, int y);
	void ProcessInput(float dt);
	void Update(float dt);
	void Render();
	void DoCollisions();
};

#endif
