/*
 * @Author: tom: https://github.com/TOMsworkspace
 * @Date: 2021-09-03 15:32:52
 * @Last Modified by: tom: https://github.com/TOMsworkspace
 * @Last Modified time: 2021-09-03 19:17:13
 */

#include <algorithm>
#include <sstream>
#include <iostream>
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


int dim = 2;
int n_node_x = 50;
int n_node_y = 6;
float node_mass = 1.0f;
int n_node = n_node_x * n_node_y;
int n_fem_element = (n_node_x - 1) * (n_node_y - 1) * 2;
float deltat = 3e-4;
float deltax = (1.0 / 32);

float Young_E = 31000.0f; /**< 杨氏模量 */
float Poisson_r = 0.3f; /**< 泊松比 [0 - 0.5] */
float Lame_parameter_1 = Young_E / (2 * (1 + Poisson_r));
float Lame_parameter_2 = Young_E * Poisson_r / ((1 + Poisson_r) * (1 - 2 * Poisson_r));
float element_v = 0.02f; /**< 微元体积 */

float radius = 0.05;

int iterate_time = 3;

inline int mesh(int i, int j) {
	return i * n_node_y + j;
}


dtkFemSimulation::dtkFemSimulation(unsigned int width, unsigned int height)
	: dtkScene(width, height),
	sphere(dtk::dtkGraphicsKernel::Point2(0.5, 0.25), radius),
	spherecenter(0.5, 0.25), rectangle(n_node_x, n_node_y)
{

}

dtkFemSimulation::~dtkFemSimulation()
{
}

void dtkFemSimulation::Init()
{
	dtkScene::Init();
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
	if (this->State == SCENE_ACTIVE) {

		// 迭代多轮, 防止穿透
		for (int i = 0; i < iterate_time; ++i) {
			//this->pre_total_energy = total_energy; 

			rectangle.compute_total_energy();
			DoCollisions();
			rectangle.compute_force();

			for (int j = 0; j < n_node; ++j) {
				rectangle.points_v_[j] = (rectangle.points_v_[j] + (rectangle.points_force_[j] / rectangle.node_mass_) * deltat) * exp(deltat * -3);

				rectangle.points_[j] += deltat * rectangle.points_v_[j];
			}

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

	Vector2f center = spherecenter;
	//Vector2f(this->sphere.center()[0], this->sphere.center()[1]);

	glColor3f(0x06 * 1.0 / 0xff, 0x85 * 1.0 / 0xff, 0x87 * 1.0 / 0xff);
	glBegin(GL_POLYGON);

	int n = 100;
	for (int i = 0; i < n; i++)
	{
		glVertex2f(center[0] + radius * cos(2 * dtk::dtkPI / n * i), center[1] + radius * sin(2 * dtk::dtkPI / n * i));
	}
	glEnd();


	//TODO: draw fem element(triangles here)
	glColor3f(0x4f * 1.0 / 0xff, 0xb9 * 1.0 / 0xff, 0x9f * 1.0 / 0xff);
	glBegin(GL_LINES);
	for (int i = 0; i < n_fem_element; ++i) {
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
	if (this->State == SCENE_ACTIVE) {
		Vector2f center = spherecenter;
		//Vector2f(this->sphere.center()[0], this->sphere.center()[1]);
		float radius = this->sphere.squared_radius();

		for (int i = 0; i < n_node; ++i) {
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
				rectangle.points_v_[i][1] = 0.0f;
			}

			if (rectangle.points_[i][1] > 0.9f) {
				rectangle.points_[i][1] = 0.9f;
				rectangle.points_v_[i][1] = 0.0f;
			}

			if (rectangle.points_[i][0] < 0.0f) {
				rectangle.points_[i][0] = 0.0f;
				rectangle.points_v_[i][0] = 0.0f;
			}

			if (rectangle.points_[i][0] > 1.0f) {
				rectangle.points_[i][0] = 1.0f;
				rectangle.points_v_[i][0] = 0.0f;
			}
		}

	}
}

void dtkFemSimulation::moveBall(int x, int y) {

	//this->spherecenter = Vector2f((x) * 1.0 / 800 , (600 - y) * 1.0 / 600 );
}