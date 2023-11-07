
/**
 * @file Particle.cpp
 * @brief Particle 实现
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

#include "Particle.h"
#include "Constants.h"

Particle::Particle() { Particle(Vector2f()); }

Particle::Particle(Eigen::Vector2f pos) {
  position = pos;
  velocity = Eigen::Vector2f(0, 0);
  force = Eigen::Vector2f(0, 0);

  mass = Constants::PARTICLE_MASS;

  density = 0;
  pressure = 0;

  color = 0;
  normal = Eigen::Vector2f();
}

float Particle::getVelocityLength2() const {
  return velocity[0] * velocity[0] + velocity[1] * velocity[1];
}

float Particle::getForceLength2() const {
  return force[0] * force[0] + force[1] * force[1];
}

float Particle::getNormalLength2() const {
  return normal[0] * normal[0] + normal[0] * normal[0];
}
