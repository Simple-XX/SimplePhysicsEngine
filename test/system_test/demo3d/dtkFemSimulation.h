
/**
 * @file dtkFemSimulation.h
 * @brief dtkFemSimulation 头文件
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
 * @Date: 2021-09-03 16:12:05
 * @Last Modified by: tom: https://github.com/TOMsworkspace
 * @Last Modified time: 2021-09-03 17:22:45
 */

#ifndef SIMPLEPHYSICSENGINE_DTKFEMSIMULATION_H
#define SIMPLEPHYSICSENGINE_DTKFEMSIMULATION_H

#include "dtkScene.h"
#include <iostream>
#include <vector>

#include <Eigen/Dense>
using namespace std;
using namespace Eigen;

class Circle2d {
public:
  double x, y, radius;
  Circle2d() : x(0), y(0), radius(0) {}
  Circle2d(double px, double py, double praius)
      : x(px), y(py), radius(praius) {}
};

class Sphere3d {
public:
  double x, y, z, radius;
  Sphere3d() : x(0), y(0), z(0), radius(0) {}
  Sphere3d(double px, double py, double pz, double praius)
      : x(px), y(py), z(pz), radius(praius) {}
};

class dtkFemSimulation : public dtkScene {

private:
  /* data */
  vector<Vector3f> pre_points; //(n_node);  /**< 点之前的位置 */

  vector<Eigen::Vector3f> points_force;

  vector<Vector3f> points_v; //(n_node); /**< 点的速度 */

  vector<Eigen::Matrix3f> B; //(n_fem_element); /**< 微元本身长度的逆 */

  float total_energy;     /** 总势能 */
  float pre_total_energy; /** 之前的总势能 */

  std::vector<std::vector<int>>
      PyramidTable; //(n_fem_element, std::vector<int>(3,0)); /**<
                    //四面体微元下标 */

  Matrix3f compute_D(int i);

  Matrix3f compute_P(int i);

  void compute_B();

  void compute_force();

  void compute_total_energy();

public:
  vector<Eigen::Vector3f> points; //(n_node);  /**< 点的位置 */
  vector<vector<long long>> mesh_index_list;
  Sphere3d sphere; //(dtk::dtkGraphicsKernel::Point2(0.5,0.2), 0.1);

  dtkFemSimulation(unsigned int width, unsigned int height);
  ~dtkFemSimulation();
  void Init();
  void InitShell();

  float getEnergy();
  // loop

  void moveBall(int x, int y);
  void ProcessInput(float dt);
  void Update(float dt);
  void Render();
  void DoCollisions();
};

#endif /* SIMPLEPHYSICSENGINE_DTKFEMSIMULATION_H */
