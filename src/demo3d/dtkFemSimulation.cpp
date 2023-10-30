/*
 * @Author: tom: https://github.com/TOMsworkspace
 * @Date: 2021-09-03 15:32:52
 * @Last Modified by: tom: https://github.com/TOMsworkspace
 * @Last Modified time: 2021-09-03 19:17:13
 */

#include <algorithm>
#include <sstream>
#include <iostream>
#include <math.h>

#include <Eigen/Dense>
using namespace std;
using namespace Eigen;


//#include <gl/GL.h>
//#include <glad/glad.h>
//#include <GLFW/glfw3.h>


//#include <irrklang/irrKlang.h>
//using namespace irrklang;

#include "dtkFemSimulation.h"

#include "GL/freeglut.h"

//#include "resource_manager.h"
//#include "sprite_renderer.h"


int dim = 3;
int n_node_x = 30;
int n_node_y = 4;
int n_node_z = 4;
float node_mass = 1.0f;
int n_node = n_node_x * n_node_y * n_node_z;
int n_fem_element = (n_node_x - 1) * (n_node_y - 1) * (n_node_z - 1) * 4;
float deltat = 3e-4;
float deltax = (1.0 / 32);

float Young_E = 3000.0f; /**< 杨氏模量 */
float Poisson_r = 0.2f; /**< 泊松比 [0 - 0.5] */
float Lame_parameter_1 = Young_E / (2 * (1 + Poisson_r));
float Lame_parameter_2 = Young_E * Poisson_r / ((1 + Poisson_r) * (1 - 2 * Poisson_r));
float element_v = 0.01f; /**< 微元体积 */

float radius = 0.05;

int iterate_time = 3;

//double M_PI = acos(-1);

inline int mesh(int i, int j, int k) {
	return k * (n_node_y * n_node_x) + j * n_node_x + i;
}


dtkFemSimulation::dtkFemSimulation(unsigned int width, unsigned int height)
	: dtkScene(width, height),
	points(n_node), pre_points(n_node), points_v(n_node), points_force(n_node), B(n_fem_element),
	PyramidTable(n_fem_element, std::vector<int>(4, 0)), sphere(0.5, 0.5, 0.0, radius),
	total_energy(0), pre_total_energy(0)
{

}

Matrix3f dtkFemSimulation::compute_D(int i) {
	int a = PyramidTable[i][0];
	int b = PyramidTable[i][1];
	int c = PyramidTable[i][2];
	int d = PyramidTable[i][3];

	Matrix3f ans;
	ans(0, 0) = points[a][0] - points[d][0];
	ans(0, 1) = points[b][0] - points[d][0];
	ans(0, 2) = points[c][0] - points[d][0];

	ans(1, 0) = points[a][1] - points[d][1];
	ans(1, 1) = points[b][1] - points[d][1];
	ans(1, 2) = points[c][1] - points[d][1];

	ans(2, 0) = points[a][2] - points[d][2];
	ans(2, 1) = points[b][2] - points[d][2];
	ans(2, 2) = points[c][2] - points[d][2];
	return ans;
}


void dtkFemSimulation::compute_B() {
	for (int i = 0; i < n_fem_element; ++i) {
		this->B[i] = compute_D(i).inverse();
		//cout << setw(6) << B[i] << endl;
	}
}

Matrix3f dtkFemSimulation::compute_P(int i) {
	Matrix3f D = compute_D(i);
	Matrix3f F = D * B[i];

	Matrix3f F_T = F.transpose().inverse();

	float J = max(0.5f, F.determinant()); /**< 形变率 */

	return Lame_parameter_1 * (F - F_T) + Lame_parameter_2 * log(J) * F_T;
	//Matrix2f ans = D * this->B[i];
}

void dtkFemSimulation::compute_total_energy() {

	this->total_energy = 0.0f;
	for (int i = 0; i < n_fem_element; ++i) {
		Matrix3f D = compute_D(i);
		Matrix3f F = D * B[i];

		//NeoHooken
		float I1 = (F * F.transpose()).trace();
		float J = max(0.2f, (float)F.determinant()); /**< 形变率 */

		//cout << J << endl;

		float element_energy_density = 0.5 * Lame_parameter_1 * (I1 - dim) - Lame_parameter_1 * log(J) + 0.5 * Lame_parameter_2 * log(J) * log(J);
		this->total_energy += element_energy_density * element_v;
	}
}

dtkFemSimulation::~dtkFemSimulation()
{
}

void dtkFemSimulation::InitShell()
{
	int scale = 1;
	//面x=x_min，x=x_max
	for (int i = 0; i < n_node_y - scale; i += scale)
	{
		for (int j = 0; j < n_node_z - scale; j += scale)
		{
			mesh_index_list.push_back({ mesh(0,i,j), mesh(0,i + scale,j), mesh(0,i + scale,j + scale), mesh(0,i,j + scale) });
			mesh_index_list.push_back({ mesh(n_node_x - scale,i,j), mesh(n_node_x - scale,i,j + scale), mesh(n_node_x - scale,i + scale,j + scale), mesh(n_node_x - scale,i + scale,j) });
		}
	}

	//面y=y_min，x=y_max
	for (int i = 0; i < n_node_z - scale; i++)
	{
		for (int j = 0; j < n_node_x - scale; j++)
		{
			mesh_index_list.push_back({ mesh(j,0,i), mesh(j,0,i + scale), mesh(j + scale,0,i + scale), mesh(j + scale,0,i) });
			mesh_index_list.push_back({ mesh(j,n_node_y - scale,i), mesh(j + scale,n_node_y - scale,i), mesh(j + scale,n_node_y - scale,i + scale), mesh(j,n_node_y - scale,i + scale) });
		}
	}

	//面z=z_min，z=z_max
	for (int i = 0; i < n_node_x - scale; i++)
	{
		for (int j = 0; j < n_node_y - scale; j++)
		{
			mesh_index_list.push_back({ mesh(i,j,0), mesh(i + scale,j,0), mesh(i + scale,j + scale,0), mesh(i,j + scale,0) });
			mesh_index_list.push_back({ mesh(i,j,n_node_z - scale), mesh(i,j + scale,n_node_z - scale), mesh(i + scale,j + scale,n_node_z - scale), mesh(i + scale,j,n_node_z - scale) });
		}
	}
}

void dtkFemSimulation::Init()
{
	dtkScene::Init();
	//TODO: load shaders

	//TODO: configure shaders

	//TODO: load textures

	//TODO: set render-specific controls

	//TODO: configure Scene objects

	//build mesh
	for (int i = 0; i < n_node_x; ++i) {
		for (int j = 0; j < n_node_y; ++j) {
			for (int k = 0; k < n_node_z; ++k) {
				int idx = mesh(i, j, k);
				//this->points[idx][0] = -14 + i * deltax * 0.5;
				//this->points[idx][1] = 8 + j * deltax * 0.5 + i * deltax * 0.05;

				this->points[idx][0] = -(deltax * 0.5f * n_node_x) / 2 + sphere.x + i * deltax * 0.5f;
				this->points[idx][1] = 0.5f + j * deltax * 0.5f + i * deltax * 0.1f;
				this->points[idx][2] = -(deltax * 0.5f * n_node_z) / 2 + sphere.z + k * deltax * 0.5f;
				this->points_v[idx][0] = 0.0f;
				this->points_v[idx][1] = -1.0f;
				this->points_v[idx][2] = 0.0f;
			}
		}
	}

	//this->pre_points = points;

	for (int i = 0; i < n_node_x - 1; ++i) {
		for (int j = 0; j < n_node_y - 1; ++j) {
			for (int k = 0; k < n_node_z - 1; ++k) {
				//element id
				int eidx = (k * (n_node_y - 1) * (n_node_x - 1) + j * (n_node_x - 1) + i) * 4;
				this->PyramidTable[eidx][0] = mesh(i + 1, j, k);
				this->PyramidTable[eidx][1] = mesh(i, j, k);
				this->PyramidTable[eidx][2] = mesh(i, j, k + 1);
				this->PyramidTable[eidx][3] = mesh(i, j + 1, k);

				eidx = (k * (n_node_y - 1) * (n_node_x - 1) + j * (n_node_x - 1) + i) * 4 + 1;
				this->PyramidTable[eidx][0] = mesh(i, j + 1, k);
				this->PyramidTable[eidx][1] = mesh(i + 1, j + 1, k);
				this->PyramidTable[eidx][2] = mesh(i + 1, j + 1, k + 1);
				this->PyramidTable[eidx][3] = mesh(i + 1, j, k);

				eidx = (k * (n_node_y - 1) * (n_node_x - 1) + j * (n_node_x - 1) + i) * 4 + 2;
				this->PyramidTable[eidx][0] = mesh(i, j, k + 1);
				this->PyramidTable[eidx][1] = mesh(i + 1, j, k + 1);
				this->PyramidTable[eidx][2] = mesh(i + 1, j, k);
				this->PyramidTable[eidx][3] = mesh(i + 1, j + 1, k + 1);

				eidx = (k * (n_node_y - 1) * (n_node_x - 1) + j * (n_node_x - 1) + i) * 4 + 3;
				this->PyramidTable[eidx][0] = mesh(i + 1, j + 1, k + 1);
				this->PyramidTable[eidx][1] = mesh(i, j + 1, k + 1);
				this->PyramidTable[eidx][2] = mesh(i, j + 1, k);
				this->PyramidTable[eidx][3] = mesh(i, j, k + 1);
			}
		}
	}

	compute_B();
	InitShell();
	this->State = SCENE_ACTIVE;
	//TODO: audio
}

void dtkFemSimulation::compute_force() {
	for (int i = 0; i < n_node; ++i) {
		this->points_force[i] = Vector3f(0.0f, -10.0f * node_mass, 0.0f);
	}

	for (int i = 0; i < n_fem_element; ++i) {

		Matrix3f P = compute_P(i);
		Matrix3f H = -element_v * (P * (this->B[i].transpose()));

		Vector3f h1 = Vector3f(H(0, 0), H(1, 0), H(2, 0));
		Vector3f h2 = Vector3f(H(0, 1), H(1, 1), H(2, 1));
		Vector3f h3 = Vector3f(H(0, 2), H(1, 2), H(2, 2));

		int a = this->PyramidTable[i][0];
		int b = this->PyramidTable[i][1];
		int c = this->PyramidTable[i][2];
		int d = this->PyramidTable[i][3];

		this->points_force[a] += h1;
		this->points_force[b] += h2;
		this->points_force[c] += h3;
		this->points_force[d] += -(h1 + h2 + h3);
	}
}

void dtkFemSimulation::Update(float dt)
{
	//TODO: update objects
	//TODO: check for object collisions
	if (this->State == SCENE_ACTIVE) {
		this->total_energy = 0;
		// 迭代多轮, 防止穿透
		for (int i = 0; i < iterate_time; ++i) {
			//this->pre_total_energy = total_energy; 
			compute_total_energy();
			DoCollisions();
			compute_force();

			//cout << total_energy << endl;

			//float deltaU = this->total_energy - this->pre_total_energy;
			//deltaU = abs(deltaU) < 1e-9 ? 0 : deltaU; 
			for (int i = 0; i < n_node; ++i) {
				// update points

				//Vector2f deltaX = this->points[i] - this->pre_points[i];

				//Vector2f diffUtoX = Vector2f(deltaU / deltaX[0], deltaU / deltaX[1]);


			   // Vector2f diffUtoX = Vector2f(abs(deltaX[0]) > 1e-4 ? deltaU / deltaX[0] : 0.0f, abs(deltaX[1]) > 1e-4 ? deltaU / deltaX[1] : 0.0f);

				//diffUtoX = Vector2f(0.0f,0.0f);

				//cout << diffUtoX << endl;

			   // this->points_v[i] = (this->points_v[i] + ((- diffUtoX / node_mass) + Vector2f(0.0f, -10.0f)) * deltat) * exp(deltat * -6);

				this->points_v[i] = (this->points_v[i] + (this->points_force[i] / node_mass) * deltat) * exp(deltat * -3);
				//this->pre_points[i] = this->points[i];

				this->points[i] += deltat * this->points_v[i];
			}


			// this->pre_points = this->points;
			// this->pre_total_energy = this->total_energy;
		}
	}
}


void dtkFemSimulation::ProcessInput(float dt)
{
	dtkScene::ProcessInput(dt);
	//TODO: process input(keys)

}


inline void GLSpherePoint(Vector3f center, int n, int i, int j)
{
	glVertex3f(center[0] + radius * cos(2 * M_PI / n * i) * sin(2 * M_PI / n * j), center[1] + radius * sin(2 * M_PI / n * i) * sin(2 * M_PI / n * j), center[2] + radius * cos(2 * M_PI / n * j));
}

void dtkFemSimulation::Render()
{
	Vector3f center = Vector3f(sphere.x, sphere.y, sphere.z);

	glEnable(GL_DEPTH_TEST);
	int n = 20;
	glColor3f(157.0 / 255.0, 184.0 / 255.0, 170.0 / 255.0);
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			glBegin(GL_POLYGON);
			GLSpherePoint(center, n, i, j);
			GLSpherePoint(center, n, i + 1, j);
			GLSpherePoint(center, n, i + 1, j + 1);
			GLSpherePoint(center, n, i, j + 1);
			glEnd();
		}
	}
	n = 30;
	glColor3f(208.0 / 255.0, 211.0 / 255.0, 213.0 / 255.0);
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			glBegin(GL_LINE_LOOP);
			GLSpherePoint(center, n, i, j);
			GLSpherePoint(center, n, i + 1, j);
			GLSpherePoint(center, n, i + 1, j + 1);
			GLSpherePoint(center, n, i, j + 1);
			glEnd();
		}
	}

	glColor3f(4.0 / 255.0, 108.0 / 255.0, 144.0 / 255.0);
	int index[3] = { 0,1,3 };
	int grid_cnt = mesh_index_list.size();
	for (int i = 0; i < grid_cnt; ++i) {
		glBegin(GL_POLYGON);
		for (int j = 0; j < 4; ++j) {
			int a = this->mesh_index_list[i][j];
			glVertex3f(this->points[a][0], this->points[a][1], this->points[a][2]);
		}
		glEnd();
	}

	glColor3f(13.0 / 255.0, 151.0 / 255.0, 168.0 / 255.0);
	for (int i = 0; i < grid_cnt; ++i) {
		glBegin(GL_LINE_LOOP);
		for (int j = 0; j < 4; ++j) {
			int a = this->mesh_index_list[i][j];
			glVertex3f(this->points[a][0], this->points[a][1], this->points[a][2]);
		}
		glEnd();
	}

	glColor3f(3.0 / 255.0, 95.0 / 255.0, 146.0 / 255.0);

	glBegin(GL_LINE_LOOP);
	glVertex3f(0.0f, 0.2f, 0.5f);
	glVertex3f(1.0f, 0.2f, 0.5f);
	glVertex3f(1.0f, 0.85f, 0.5f);
	glVertex3f(0.0f, 0.85f, 0.5f);
	glEnd();

	glBegin(GL_LINE_LOOP);
	glVertex3f(0.0f, 0.2f, -0.5f);
	glVertex3f(1.0f, 0.2f, -0.5f);
	glVertex3f(1.0f, 0.85f, -0.5f);
	glVertex3f(0.0f, 0.85f, -0.5f);
	glEnd();

	glBegin(GL_LINES);
	glVertex3f(0.0f, 0.2f, -0.5f);
	glVertex3f(0.0f, 0.2f, 0.5f);

	glVertex3f(1.0f, 0.2f, -0.5f);
	glVertex3f(1.0f, 0.2f, 0.5f);

	glVertex3f(1.0f, 0.85f, -0.5f);
	glVertex3f(1.0f, 0.85f, 0.5f);

	glVertex3f(0.0f, 0.85f, -0.5f);
	glVertex3f(0.0f, 0.85f, 0.5f);
	glEnd();

}

// collision detection

void dtkFemSimulation::DoCollisions()
{
	if (this->State == SCENE_ACTIVE) {
		Vector3f center = Vector3f(sphere.x, sphere.y, sphere.z);
		//Vector2f(this->sphere.center()[0], this->sphere.center()[1]);
		float radius = this->sphere.radius;

		for (int i = 0; i < n_node; ++i) {
			//# Collide with sphere

			Vector3f dis = this->points[i] - center;
			if ((float)(dis.dot(dis)) < radius * radius)
			{
				Vector3f normal = dis.normalized();

				this->points[i] = center + radius * normal;
				this->points_v[i] -= (this->points_v[i].dot(normal)) * normal;
			}


			// Collide with ground

			if (this->points[i][2] < -0.5f) {
				this->points[i][2] = -0.5f;
				this->points_v[i][2] = 0.0f;
			}

			if (this->points[i][2] > 0.5f) {
				this->points[i][2] = 0.5f;
				this->points_v[i][2] = 0.0f;
			}

			if (this->points[i][1] < 0.2f) {
				this->points[i][1] = 0.2f;
				this->points_v[i][1] = 0.0f;
			}

			if (this->points[i][1] > 0.9f) {
				this->points[i][1] = 0.9f;
				this->points_v[i][1] = 0.0f;
			}

			if (this->points[i][0] < 0.0f) {
				this->points[i][0] = 0.0f;
				this->points_v[i][0] = 0.0f;
			}

			if (this->points[i][0] > 1.0f) {
				this->points[i][0] = 1.0f;
				this->points_v[i][0] = 0.0f;
			}
		}

	}
}

void dtkFemSimulation::moveBall(int x, int y) {

	//this->spherecenter = Vector2f((x) * 1.0 / 800 , (600 - y) * 1.0 / 600 );
}

float dtkFemSimulation::getEnergy() {
	return this->total_energy;
}