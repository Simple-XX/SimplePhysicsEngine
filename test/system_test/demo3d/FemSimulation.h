
/**
 * @file dtkFemSimulation.h
 * @brief dtkFemSimulation 头文件
 * @author tom (https://github.com/TOMsworkspace)
 * @version 1.0
 * @date 2021-09-03
 * @copyright MIT LICENSE
 * https://github.com/Simple-XX/SimplePhysicsEngine
 * @par change log:
 * <table>
 * <tr><th>Date<th>Author<th>Description
 * <tr><td>2021-09-03<td>tom<td>创建文件
 * <tr><td>2023-10-31<td>Zone.N<td>迁移到 doxygen
 * </table>
 */

#ifndef SIMPLEPHYSICSENGINE_FEMSIMULATION_H
#define SIMPLEPHYSICSENGINE_FEMSIMULATION_H

#include <iostream>
#include <vector>

#include <Eigen/Dense>
#include <dtkScene.h>

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

class FemSimulation : public dtk::dtkScene {

private:
  /* data */
  std::vector<Eigen::Vector3f> pre_points; //(n_node);  /**< 点之前的位置 */

  std::vector<Eigen::Vector3f> points_force;

  std::vector<Eigen::Vector3f> points_v; //(n_node); /**< 点的速度 */

  std::vector<Eigen::Matrix3f> B; //(n_fem_element); /**< 微元本身长度的逆 */

  float total_energy;     /** 总势能 */
  float pre_total_energy; /** 之前的总势能 */

  std::vector<std::vector<int>>
      PyramidTable; //(n_fem_element, std::vector<int>(3,0)); /**<
                    // 四面体微元下标 */

  Eigen::Matrix3f compute_D(int i);

  Eigen::Matrix3f compute_P(int i);

  void compute_B();

  void compute_force();

  void compute_total_energy();

public:
  std::vector<Eigen::Vector3f> points; //(n_node);  /**< 点的位置 */
  std::vector<std::vector<long long>> mesh_index_list;
  Sphere3d sphere; //(dtk::dtkGraphicsKernel::Point2(0.5,0.2), 0.1);

  FemSimulation(unsigned int width, unsigned int height);
  ~FemSimulation();
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

#endif /* SIMPLEPHYSICSENGINE_FEMSIMULATION_H */
