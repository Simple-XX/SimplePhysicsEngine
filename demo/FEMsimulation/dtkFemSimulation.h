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
#include <memory>
#include <unordered_map>
#include "cbody.h"
#include "cjoint.h"
#include "cpair.h"
#include "ctypes.h"

#include <Eigen/Dense>
using namespace std;
using namespace Eigen;
using namespace dtk;
using namespace clib;

class dtkFemSimulation : public dtkScene
{
private:
	dtk::dtkGraphicsKernel::Circle2 sphere;//(dtk::dtkGraphicsKernel::Point2(0.5,0.2), 0.1);
	Vector2f spherecenter;
	//刚体部分
	using body_list = std::vector<cbody::ptr>;
	using joint_list = std::vector<cjoint::ptr>;
	using pair_list = std::unordered_map<uint32_t, cpair::ptr>;

	bool _pause{ false }; // 是否暂停
	clib::vec2 _gravity; // 重力
	size_t _iterations{ 10 };

	body_list _bodies; // 刚体列表
	joint_list _joints; // 关节数组
	pair_list _arbiters; // 碰撞检测对列表


public:
	dtkMesh rectangle;
	dtkFemSimulation(unsigned int width, unsigned int height, const vec2& gravity);
	~dtkFemSimulation();
	void Init();

	void moveBall(int x, int y);
	void ProcessInput(float dt);
	void Update(float dt);
	void Render();
	void DoCollisions();

	//刚体部分
	void add(cbody::ptr body);
	void add(cjoint::ptr joint);
	const vec2& get_gravity() const;
	const body_list& get_bodies() const;
	const joint_list& get_joints() const;
	const pair_list& get_arbiters() const;

	void move(const vec2& v);

	void clear();
	void step(decimal dt);

	bool is_pause() const;
	void set_pause(bool pause);
};

class cfactory {
public:
	static polygon_body::ptr make_box(decimal mass, decimal width, decimal height,
		const vec2& position = vec2());
	static polygon_body::ptr make_polygon(decimal mass,
		const polygon_body::vertex_list& vertices,
		const vec2& position = vec2());
	static polygon_body::ptr make_fence(dtkFemSimulation& world);
	static cpair::ptr make_arbiter(cbody::ptr a, cbody::ptr b,
		const vec2& normal,
		const cpair::contact_list& contacts = cpair::contact_list());
	static revolute_joint::ptr make_revolute_joint(cbody::ptr a, cbody::ptr b,
		const vec2& anchor);
};

#endif
