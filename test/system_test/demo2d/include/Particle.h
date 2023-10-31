
/**
 * @file Particle.h
 * @brief Particle 头文件
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

#pragma once

#include <Eigen/Dense>

class Particle {
public:
  Eigen::Vector2f position;
  Eigen::Vector2f velocity;
  Eigen::Vector2f force;

  float mass;
  float density;
  float pressure;

  float color;
  Eigen::Vector2f normal;

  Particle();
  Particle(Eigen::Vector2f position);

  Eigen::Vector4f renderColor;

  float getVelocityLength2() const;
  float getForceLength2() const;
  float getNormalLength2() const;
};