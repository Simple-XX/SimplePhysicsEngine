
/**
* @file dtkFemSimulation.cpp
* @brief dtkFemSimulation 实现
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

float deltat = 0.001;

int iterate_time = 3;


dtkFemSimulation::dtkFemSimulation(const dtk::dtkDouble2& gravity)
	:spherecenter(0.4, 0.5), _gravity(gravity)
{
	sphere.x = spherecenter[0];
	sphere.y = spherecenter[1];
}

void dtkFemSimulation::add(dtk::dtkRigidBody::ptr body) { _bodies.push_back(body); }

void dtkFemSimulation::add(dtk::dtkJoint::ptr joint) { _joints.push_back(joint); }

void dtkFemSimulation::add(dtkMesh::ptr mesh) { mesh->Init(); _meshes.push_back(mesh); }

void dtkFemSimulation::add(SPHSolver::ptr sph) { _sphs.push_back(sph); }

const dtk::dtkDouble2& dtkFemSimulation::get_gravity() const { return _gravity; }

const dtkFemSimulation::body_list& dtkFemSimulation::get_bodies() const { return _bodies; }

const dtkFemSimulation::joint_list& dtkFemSimulation::get_joints() const { return _joints; }

const dtkFemSimulation::pair_list& dtkFemSimulation::get_arbiters() const { return _arbiters; }

const dtkFemSimulation::mesh_list& dtkFemSimulation::get_meshes() const { return _meshes; }

const dtkFemSimulation::sph_list& dtkFemSimulation::get_sphs() const { return _sphs; }

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
	int body_cnt = _bodies.size();
	for (size_t i = 0; i < body_cnt; ++i) {
		for (size_t j = i + 1; j < body_cnt; ++j) {
			auto a = std::dynamic_pointer_cast<dtk::dtkPolygonRigidBody>(_bodies[i]);
			auto b = std::dynamic_pointer_cast<dtk::dtkPolygonRigidBody>(_bodies[j]);
			if (!a->can_collide(*b)) {
				continue;
			}
			uint32_t id;
			auto arbiter = dtk::dtkCollisionPair::is_collide_rr(a, b, id);
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

	// 更新重力
	for (auto& body : _bodies) {
		body->update_force(_gravity, dt);
	}

	//更新mesh运动
	for (auto& mesh : _meshes) {
		for (int i = 0; i < iterate_time; ++i) {
			for (auto& body : _bodies)
			{
				auto poly_body = std::dynamic_pointer_cast<dtk::dtkPolygonRigidBody>(body);
				dtk::dtkCollisionPair::do_collision_mr(mesh, poly_body);
			}
			mesh->compute_force();

			for (int j = 0; j < (*mesh).n_node_; ++j) {
				(*mesh).points_v_[j] = ((*mesh).points_v_[j] + ((*mesh).points_force_[j] / (*mesh).node_mass_) * deltat) * exp(deltat * -3);

				(*mesh).points_[j] += deltat * (*mesh).points_v_[j];
			}

		}
		mesh->updateShell();
	}
	//更新sph
	for (auto& sph : _sphs) {
		sph->update(Constants::TIMESTEP);
	}

}

void dtkFemSimulation::clear() {
	_arbiters.clear();
	_joints.clear();
	_bodies.clear();
	_meshes.clear();
	_sphs.clear();
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

dtkMesh::ptr dtkFactory::make_mesh(int n_x_node, int n_y_node, const dtk::dtkDouble2& position)
{
	return std::make_shared<dtkMesh>(global_id++, n_x_node, n_y_node, Vector2f(position.x, position.y));
}

SPHSolver::ptr dtkFactory::make_sph()
{
	return std::make_shared<SPHSolver>();
}
