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
int n_node_x = 50;
int n_node_y = 6;
int n_node_z = 6;
float node_mass = 1.0f;
int n_node = n_node_x * n_node_y * n_node_z;
int n_fem_element = (n_node_x - 1) * (n_node_y - 1) * (n_node_z - 1) * 4;
float deltat = 3e-4;
float deltax = (1.0 / 32);

float Young_E = 1000.0f; /**< 杨氏模量 */
float Poisson_r = 0.3f; /**< 泊松比 [0 - 0.5] */
float Lame_parameter_1 = Young_E / (2 * (1 + Poisson_r));
float Lame_parameter_2 = Young_E * Poisson_r / ((1 + Poisson_r) * (1 - 2 * Poisson_r));
float element_v = 0.01f; /**< 微元体积 */

float radius = 0.05;

int iterate_time = 3;

double M_PI = acos(-1);

inline int mesh(int i, int j, int k) {
	return k * (n_node_y * n_node_x) + j * n_node_x + i;
}


dtkFemSimulation::dtkFemSimulation(unsigned int width, unsigned int height)
	: dtkScene(width, height),
	points(n_node), pre_points(n_node), points_v(n_node), points_force(n_node), B(n_fem_element),
	PyramidTable(n_fem_element, std::vector<int>(4, 0)), sphere(0.5, 0.25, 0.0, radius),
	total_energy(0), pre_total_energy(0), spherecenter(0.5, 0.25, 0.0)
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

				this->points[idx][0] = 0.1f + i * deltax * 0.5f;
				this->points[idx][1] = 0.5f + j * deltax * 0.5f + i * deltax * 0.1f;
				this->points[idx][2] = k * deltax * 0.5f;
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


void dtkFemSimulation::Render()
{
	//if(this->State == SCENE_ACTIVE){
		//TODO: draw circle

	Vector3f center = spherecenter;
	//Vector2f(this->sphere.center()[0], this->sphere.center()[1]);

	glColor3f(0x06 * 1.0 / 0xff, 0x85 * 1.0 / 0xff, 0x87 * 1.0 / 0xff);
	glBegin(GL_POLYGON);

	int n = 100;
	for (int i = 0; i < n; i++)
	{
		glVertex2f(center[0] + radius * cos(2 * M_PI / n * i), center[1] + radius * sin(2 * M_PI / n * i));
	}
	glEnd();


	//TODO: draw fem element(triangles here)
	glColor3f(0x4f * 1.0 / 0xff, 0xb9 * 1.0 / 0xff, 0x9f * 1.0 / 0xff);
	glBegin(GL_LINES);
	int index[3] = { 0,1,3 };
	for (int i = 0; i < n_fem_element; ++i) {
		for (int j = 0; j < 3; ++j) {
			int a = this->PyramidTable[i][index[j]];
			int b = this->PyramidTable[i][index[(j + 1) % 3]];

			//draw line from a to b;
			glVertex2f(this->points[a][0], this->points[a][1]);
			glVertex2f(this->points[b][0], this->points[b][1]);
		}
	}
	//glEnd();

	glColor3f(1.0, 1.0, 1.0);
	//glBegin(GL_LINES);
	glVertex2f(0.0f, 0.2f);
	glVertex2f(1.0f, 0.2f);

	glVertex2f(1.0f, 0.85f);
	glVertex2f(1.0f, 0.2f);

	glVertex2f(1.0f, 0.85f);
	glVertex2f(0.0f, 0.85f);

	glVertex2f(0.0f, 0.85f);
	glVertex2f(0.0f, 0.2f);
	glEnd();
	// }
}

// collision detection

void dtkFemSimulation::DoCollisions()
{
	if (this->State == SCENE_ACTIVE) {
		Vector3f center = spherecenter;
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