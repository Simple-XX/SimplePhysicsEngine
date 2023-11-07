
/**
 * @file SPHSolver.h
 * @brief SPHSolver 头文件
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

#ifndef SIMPLEPHYSICSENGINE_SPHCOLVER_H
#define SIMPLEPHYSICSENGINE_SPHCOLVER_H

#include <memory>
#include <vector>

#include "Grid.h"
#include "Particle.h"

enum class Visualization { Default, Velocity, Force, Density, Pressure, Water };

class SPHSolver {
public:
  using ptr = std::shared_ptr<SPHSolver>;
  SPHSolver();

  void update(float dt);

  void repulsionForce(Eigen::Vector2f position);
  void attractionForce(Eigen::Vector2f position);
  std::vector<Particle> particles;

private:
  int numberParticles;

  std::vector<std::vector<int>> neighborhoods;
  Grid grid;

  float kernel(Eigen::Vector2f x, float h);
  Eigen::Vector2f gradKernel(Eigen::Vector2f x, float h);
  float laplaceKernel(Eigen::Vector2f x, float h);

  void findNeighborhoods();

  void calculateDensity();
  void calculatePressure();

  void calculateForceDensity();

  void integrationStep(float dt);

  void collisionHandling();
};

#endif /* SIMPLEPHYSICSENGINE_SPHCOLVER_H */
