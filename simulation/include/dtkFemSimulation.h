﻿/*
 * @Author: tom: https://github.com/TOMsworkspace
 * @Date: 2021-09-03 16:12:05
 * @Last Modified by: tom: https://github.com/TOMsworkspace
 * @Last Modified time: 2021-09-03 17:22:45
 */



#include "dtkScene.h"
#include "dtkMesh.h"
#include <iostream>
#include <vector>
#include <memory>
#include <unordered_map>
#include "dtkCollisionPair.h"
#include "dtkJoint.h"

#include <Eigen/Dense>
using namespace std;
using namespace Eigen;

struct Circle2
{
	double x, y, radius;
};

class dtkFemSimulation
{
public:
	dtkMesh rectangle;
	dtkFemSimulation(const dtk::dtkDouble2& gravity);
	~dtkFemSimulation() = default;
	void Init();

	void moveBall(int x, int y);
	void ProcessInput(float dt);
	void Update(float dt);
	void Render();
	void DoCollisions();

	//刚体部分
	using body_list = std::vector<dtk::dtkRigidBody::ptr>;
	using joint_list = std::vector<dtk::dtkJoint::ptr>;
	using pair_list = std::unordered_map<uint32_t, dtk::dtkCollisionPair::ptr>;

	void add(dtk::dtkRigidBody::ptr body);
	void add(dtk::dtkJoint::ptr joint);
	const dtk::dtkDouble2& get_gravity() const;
	const body_list& get_bodies() const;
	const joint_list& get_joints() const;
	const pair_list& get_arbiters() const;

	void move(const dtk::dtkDouble2& v);

	void clear();
	void step(double dt);

	bool is_pause() const;
	void set_pause(bool pause);


private:
	Circle2 sphere;
	Vector2f spherecenter;
	//刚体部分

	bool _pause{ false }; // 是否暂停
	dtk::dtkDouble2 _gravity; // 重力
	size_t _iterations{ 10 };

	body_list _bodies; // 刚体列表
	joint_list _joints; // 关节数组
	pair_list _arbiters; // 碰撞检测对列表
};

class dtkFactory {
public:
	static dtk::dtkPolygonRigidBody::ptr make_box(double mass, double width, double height,
		const dtk::dtkDouble2& position = dtk::dtkDouble2());
	static dtk::dtkPolygonRigidBody::ptr make_polygon(double mass,
		const dtk::dtkPolygonRigidBody::vertex_list& vertices,
		const dtk::dtkDouble2& position = dtk::dtkDouble2());
	static dtk::dtkPolygonRigidBody::ptr make_fence(dtkFemSimulation& world);
	static dtk::dtkCollisionPair::ptr make_arbiter(dtk::dtkRigidBody::ptr a, dtk::dtkRigidBody::ptr b,
		const dtk::dtkDouble2& normal,
		const dtk::dtkCollisionPair::contact_list& contacts = dtk::dtkCollisionPair::contact_list());
	static dtk::dtkRevoluteJoint::ptr make_revolute_joint(dtk::dtkRigidBody::ptr a, dtk::dtkRigidBody::ptr b,
		const dtk::dtkDouble2& anchor);
};
