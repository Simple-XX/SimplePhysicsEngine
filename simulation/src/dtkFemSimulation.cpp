/*
 * @Author: tom: https://github.com/TOMsworkspace
 * @Date: 2021-09-03 15:32:52
 * @Last Modified by: tom: https://github.com/TOMsworkspace
 * @Last Modified time: 2021-09-03 19:17:13
 */
 /*
 #include <algorithm>
 #include <sstream>
 #include <iostream>
 #include <Eigen/Dense>
 using namespace std;
 using namespace Eigen;*/

#include "dtkFemSimulation.h"

#include "GL/freeglut.h"

int n_node_x = 50;
int n_node_y = 6;
float deltat = 0.0001;

float radius = 0.05;

int iterate_time = 3;


dtkFemSimulation::dtkFemSimulation(const dtk::dtkDouble2& gravity)
	:spherecenter(0.4, 0.5), rectangle(n_node_x, n_node_y), _gravity(gravity)
{
	sphere.x = spherecenter[0];
	sphere.y = spherecenter[1];
	sphere.radius = radius;
}

void dtkFemSimulation::Init()
{
	rectangle.Init();
	//TODO: load shaders

	//TODO: configure shaders

	//TODO: load textures

	//TODO: set render-specific controls

	//TODO: configure Scene objects

	//build mesh

	//TODO: audio
}

/*
void dtkFemSimulation::Update(float dt)
{
	//TODO: update objects
	//TODO: check for object collisions
	if (this->State == SCENE_ACTIVE) {

		// 迭代多轮, 防止穿透
		for (int i = 0; i < iterate_time; ++i) {
			//this->pre_total_energy = total_energy;
			compute_total_energy();
			DoCollisions();
			compute_force();

			//cout << total_energy << endl;

			//float deltaU = this->total_energy - this->pre_total_energy;
			//deltaU = abs(deltaU) < 1e-9 ? 0 : deltaU;
			for (int j = 0; j < n_node; ++j) {
				// update points

				//Vector2f deltaX = this->points[i] - this->pre_points[i];

				//Vector2f diffUtoX = Vector2f(deltaU / deltaX[0], deltaU / deltaX[1]);


			   // Vector2f diffUtoX = Vector2f(abs(deltaX[0]) > 1e-4 ? deltaU / deltaX[0] : 0.0f, abs(deltaX[1]) > 1e-4 ? deltaU / deltaX[1] : 0.0f);

				//diffUtoX = Vector2f(0.0f,0.0f);

				//cout << diffUtoX << endl;

			   // this->points_v[i] = (this->points_v[i] + ((- diffUtoX / node_mass) + Vector2f(0.0f, -10.0f)) * deltat) * exp(deltat * -6);

				this->points_v[j] = (this->points_v[j] + (this->points_force[j] / node_mass) * deltat) * exp(deltat * -3);
				//this->pre_points[i] = this->points[i];

				this->points[j] += deltat * this->points_v[j];
			}


			// this->pre_points = this->points;
			// this->pre_total_energy = this->total_energy;
		}
	}
}*/

void dtkFemSimulation::Update(float dt)
{
	//TODO: update objects
	//TODO: check for object collisions

		// 迭代多轮, 防止穿透
	for (int i = 0; i < iterate_time; ++i) {
		//this->pre_total_energy = total_energy; 

		rectangle.compute_total_energy();
		DoCollisions();
		rectangle.compute_force();

		for (int j = 0; j < rectangle.n_node_; ++j) {
			rectangle.points_v_[j] = (rectangle.points_v_[j] + (rectangle.points_force_[j] / rectangle.node_mass_) * deltat) * exp(deltat * -3);

			rectangle.points_[j] += deltat * rectangle.points_v_[j];
		}

	}
}

void dtkFemSimulation::ProcessInput(float dt)
{
	//dtkScene::ProcessInput(dt);
	//TODO: process input(keys)

}

void dtkFemSimulation::Render()
{
	//if(this->State == SCENE_ACTIVE){
		//TODO: draw circle

	Vector2f center = spherecenter;
	//Vector2f(this->sphere.center()[0], this->sphere.center()[1]);

	glColor3f(0x06 * 1.0 / 0xff, 0x85 * 1.0 / 0xff, 0x87 * 1.0 / 0xff);
	glBegin(GL_POLYGON);

	int n = 100;
	for (int i = 0; i < n; i++)
	{
		glVertex2f(center[0] + radius * cos(2 * 3.1415 / n * i), center[1] + radius * sin(2 * 3.1415 / n * i));
	}
	glEnd();


	//TODO: draw fem element(triangles here)
	glColor3f(0x4f * 1.0 / 0xff, 0xb9 * 1.0 / 0xff, 0x9f * 1.0 / 0xff);
	glBegin(GL_LINES);
	for (int i = 0; i < rectangle.n_fem_element_; ++i) {
		for (int j = 0; j < 3; ++j) {
			int a = rectangle.mesh_table_[i][j];
			int b = rectangle.mesh_table_[i][(j + 1) % 3];

			//draw line from a to b;
			glVertex2f(rectangle.points_[a][0], rectangle.points_[a][1]);
			glVertex2f(rectangle.points_[b][0], rectangle.points_[b][1]);
		}
	}
	glEnd();


}

// collision detection

void dtkFemSimulation::DoCollisions()
{
	Vector2f center = spherecenter;
	//Vector2f(this->sphere.center()[0], this->sphere.center()[1]);
	float radius = this->sphere.radius;

	for (int i = 0; i < rectangle.n_node_; ++i) {
		//# Collide with sphere

		Vector2f dis = rectangle.points_[i] - center;
		if ((float)(dis.dot(dis)) < radius * radius)
		{
			Vector2f normal = dis.normalized();

			rectangle.points_[i] = center + radius * normal;
			rectangle.points_v_[i] -= (rectangle.points_v_[i].dot(normal)) * normal;
		}


		// Collide with ground

		if (rectangle.points_[i][1] < 0.2f) {
			rectangle.points_[i][1] = 0.2f;
			rectangle.points_v_[i][1] *= -0.5f;
		}

		if (rectangle.points_[i][1] > 0.9f) {
			rectangle.points_[i][1] = 0.9f;
			rectangle.points_v_[i][1] *= -0.5f;
		}

		if (rectangle.points_[i][0] < 0.0f) {
			rectangle.points_[i][0] = 0.0f;
			rectangle.points_v_[i][0] *= -0.5f;
		}

		if (rectangle.points_[i][0] > 1.0f) {
			rectangle.points_[i][0] = 1.0f;
			rectangle.points_v_[i][0] *= -0.5f;
		}
	}

}

void dtkFemSimulation::moveBall(int x, int y) {

	//this->spherecenter = Vector2f((x) * 1.0 / 800 , (600 - y) * 1.0 / 600 );
}

void dtkFemSimulation::add(dtk::dtkRigidBody::ptr body) { _bodies.push_back(body); }

void dtkFemSimulation::add(dtk::dtkJoint::ptr joint) { _joints.push_back(joint); }

const dtk::dtkDouble2& dtkFemSimulation::get_gravity() const { return _gravity; }

const dtkFemSimulation::body_list& dtkFemSimulation::get_bodies() const { return _bodies; }

const dtkFemSimulation::joint_list& dtkFemSimulation::get_joints() const { return _joints; }

const dtkFemSimulation::pair_list& dtkFemSimulation::get_arbiters() const {
	return _arbiters;
}

void dtkFemSimulation::move(const dtk::dtkDouble2& v) {
	for (auto& body : _bodies) {
		if (body->get_inv_mass() > 0)
			body->update_impulse(v * body->get_inv_mass(), dtk::dtkDouble2());
	}
}

void dtkFemSimulation::step(double dt) {
	if (_pause)
		return;
	// 碰撞检测
	for (size_t i = 0; i < _bodies.size(); ++i) {
		for (size_t j = i + 1; j < _bodies.size(); ++j) {
			auto a = std::dynamic_pointer_cast<dtk::dtkPolygonRigidBody>(_bodies[i]);
			auto b = std::dynamic_pointer_cast<dtk::dtkPolygonRigidBody>(_bodies[j]);
			if (!a->can_collide(*b)) {
				continue;
			}
			uint32_t id;
			auto arbiter = dtk::dtkCollisionPair::is_collide(a, b, id);
			auto iter = _arbiters.find(id);
			if (arbiter == nullptr) {
				if (iter != _arbiters.end()) {
					_arbiters.erase(id);
				}
			}
			else if (iter == _arbiters.end()) {
				_arbiters[id] = arbiter;
			}
			else {
				auto& old_arbiter = iter->second;
				arbiter->update(*old_arbiter);
				_arbiters[id] = arbiter;
			}
		}
	}
	for (auto& kv : _arbiters) {
		kv.second->pre_step(dt);
	}
	for (auto& joint : _joints) {
		joint->pre_step(dt);
	}

	// 更新动量
	for (size_t i = 0; i < _iterations; ++i) {
		for (auto& kv : _arbiters) {
			kv.second->update_impulse();
		}
		for (auto& joint : _joints) {
			joint->update_impulse();
		}
	}

	// 更新外力
	for (auto& body : _bodies) {
		body->update_force(_gravity, dt);
	}
}

void dtkFemSimulation::clear() {
	_arbiters.clear();
	_joints.clear();
	_bodies.clear();
}

bool dtkFemSimulation::is_pause() const {
	return _pause;
}

void dtkFemSimulation::set_pause(bool pause) {
	_pause = pause;
}


//工厂
static uint16_t global_id = 1;

dtk::dtkPolygonRigidBody::ptr dtkFactory::make_box(double mass, double width, double height, const dtk::dtkDouble2& position) {
	// 四边形
	// 注意顶点呈逆时针
	dtk::dtkPolygonRigidBody::vertex_list vertices = {
		{width / 2,  height / 2},
		{-width / 2, height / 2},
		{-width / 2, -height / 2},
		{width / 2,  -height / 2}
	};
	auto body = std::make_shared<dtk::dtkPolygonRigidBody>(global_id++, mass, vertices);
	body->set_position(position);
	return body;
}

dtk::dtkPolygonRigidBody::ptr
dtkFactory::make_polygon(double mass, const dtk::dtkPolygonRigidBody::vertex_list& vertices, const dtk::dtkDouble2& position) {
	auto body = std::make_shared<dtk::dtkPolygonRigidBody>(global_id++, mass, vertices);
	body->set_position(position);
	return body;
}

dtk::dtkPolygonRigidBody::ptr dtkFactory::make_fence(dtkFemSimulation& world) {
	auto ground = make_box(dtk::inf, 20, 1, { 0, -0.5 });
	world.add(ground);
	world.add(make_box(dtk::inf, 20, 1, { 0, 16.5 }));
	world.add(make_box(dtk::inf, 1, 18, { -9.5, 8 }));
	world.add(make_box(dtk::inf, 1, 18, { 9.5, 8 }));
	return ground;
}

dtk::dtkCollisionPair::ptr
dtkFactory::make_arbiter(dtk::dtkRigidBody::ptr a, dtk::dtkRigidBody::ptr b, const dtk::dtkDouble2& normal, const dtk::dtkCollisionPair::contact_list& contacts) {
	return std::make_shared<dtk::dtkCollisionPair>(a, b, normal, contacts);
}

dtk::dtkRevoluteJoint::ptr dtkFactory::make_revolute_joint(dtk::dtkRigidBody::ptr a, dtk::dtkRigidBody::ptr b, const dtk::dtkDouble2& anchor) {
	return std::make_shared<dtk::dtkRevoluteJoint>(a, b, anchor);
}
