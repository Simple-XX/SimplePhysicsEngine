
/**
 * @file SPHSolver.cpp
 * @brief SPHSolver 实现
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

#include <cmath>
#include <iostream>

#include "Constants.h"
#include "SPHSolver.h"

SPHSolver::SPHSolver() {
  int particlesX = Constants::NUMBER_PARTICLES / 2.0f;
  int particlesY = Constants::NUMBER_PARTICLES;

  numberParticles = particlesX * particlesY;
  particles = std::vector<Particle>();

  float width = Constants::WIDTH / 4.2f;
  float height = 2.0f * Constants::HEIGHT / 4.0f;

  float dx = width / particlesX;
  float dy = height / particlesY;

  for (int i = 0; i < Constants::NUMBER_PARTICLES / 2.0f; i++) {
    for (int j = 0; j < Constants::NUMBER_PARTICLES; j++) {
      Eigen::Vector2f pos = Eigen::Vector2f((Constants::WIDTH - width) / 2,
                                            (Constants::HEIGHT - height) / 2) +
                            Eigen::Vector2f(i * dx, j * dy);

      Particle p = Particle(pos);
      particles.push_back(p);
    }
  }

  grid.updateStructure(particles);

  std::cout << "SPH Solver initialized with " << numberParticles
            << " particles." << std::endl;
}

void SPHSolver::repulsionForce(Eigen::Vector2f position) {
  for (int i = 0; i < numberParticles; i++) {
    Eigen::Vector2f x = particles[i].position - position;
    // Vector2f x = Vector2f(x1[0], x1[1]);
    float dist2 = x[0] * x[0] + x[1] * x[1];

    if (dist2 < Constants::KERNEL_RANGE * 3) {
      particles[i].force += x * 800000.0f * particles[i].density;
    }
  }
}

void SPHSolver::attractionForce(Eigen::Vector2f position) {
  for (int i = 0; i < numberParticles; i++) {

    Eigen::Vector2f x = position - particles[i].position;
    // Vector2f x = Vector2f(x1[0], x1[1]);
    float dist2 = x[0] * x[0] + x[1] * x[1];

    if (dist2 < Constants::KERNEL_RANGE * 3) {
      particles[i].force += x * 800000.0f * particles[i].density;
    }
  }
}

void SPHSolver::update(float dt) {
  for (int i = 0; i < particles.size(); i++) {
    particles[i].force = Eigen::Vector2f(0.0, 0.0);
  }
  findNeighborhoods();
  calculateDensity();
  calculatePressure();
  calculateForceDensity();
  integrationStep(dt);
  collisionHandling();

  grid.updateStructure(particles);
}

// Poly6 Kernel
float SPHSolver::kernel(Eigen::Vector2f x, float h) {
  float r2 = x[0] * x[0] + x[1] * x[1];
  float h2 = h * h;

  if (r2 < 0 || r2 > h2)
    return 0.0f;

  return 315.0f / (64.0f * M_PI * pow(h, 9)) * pow(h2 - r2, 3);
}

// Gradient of Spiky Kernel
Eigen::Vector2f SPHSolver::gradKernel(Eigen::Vector2f x, float h) {
  float r = sqrt(x[0] * x[0] + x[1] * x[1]);
  if (r == 0.0f)
    return Eigen::Vector2f(0.0f, 0.0f);

  float t1 = -45.0f / (M_PI * pow(h, 6));
  Eigen::Vector2f t2 = x / r;
  float t3 = pow(h - r, 2);

  return t1 * t2 * t3;
}

// Laplacian of Viscosity Kernel
float SPHSolver::laplaceKernel(Eigen::Vector2f x, float h) {
  float r = sqrt(x[0] * x[0] + x[1] * x[1]);
  return 45.0f / (M_PI * pow(h, 6)) * (h - r);
}

void SPHSolver::findNeighborhoods() {
  neighborhoods = std::vector<std::vector<int>>();
  float maxDist2 = Constants::KERNEL_RANGE * Constants::KERNEL_RANGE;

  for (const Particle &p : particles) {
    std::vector<int> neighbors = std::vector<int>();
    std::vector<Cell> neighboringCells = grid.getNeighboringCells(p.position);

    for (const Cell &cell : neighboringCells) {
      for (int index : cell) {
        Eigen::Vector2f x = p.position - particles[index].position;
        float dist2 = x[0] * x[0] + x[1] * x[1];
        if (dist2 <= maxDist2) {
          neighbors.push_back(index);
        }
      }
    }

    neighborhoods.push_back(neighbors);
  }
}

void SPHSolver::calculateDensity() {
  for (int i = 0; i < numberParticles; i++) {
    std::vector<int> neighbors = neighborhoods[i];
    float densitySum = 0.0f;

    for (int n = 0; n < neighbors.size(); n++) {
      int j = neighbors[n];
      Eigen::Vector2f x = particles[i].position - particles[j].position;

      densitySum += particles[j].mass * kernel(x, Constants::KERNEL_RANGE);
    }

    particles[i].density = densitySum;
  }
}

void SPHSolver::calculatePressure() {
  for (int i = 0; i < numberParticles; i++) {
    particles[i].pressure = std::max(
        Constants::STIFFNESS * (particles[i].density - Constants::REST_DENSITY),
        0.0f);
  }
}

void SPHSolver::calculateForceDensity() {
  for (int i = 0; i < numberParticles; i++) {
    Eigen::Vector2f fPressure = Eigen::Vector2f(0.0f, 0.0f);
    Eigen::Vector2f fViscosity = Eigen::Vector2f(0.0f, 0.0f);
    Eigen::Vector2f fGravity = Eigen::Vector2f(0.0f, 0.0f);

    std::vector<int> neighbors = neighborhoods[i];

    // particles[i].color = 0;

    for (int n = 0; n < neighbors.size(); n++) {
      int j = neighbors[n];
      Eigen::Vector2f x = particles[i].position - particles[j].position;

      // Pressure force density
      Eigen::Vector2f res = gradKernel(x, Constants::KERNEL_RANGE);
      fPressure += particles[j].mass *
                   (particles[i].pressure + particles[j].pressure) /
                   (2.0f * particles[j].density) * res;

      // Viscosity force density
      fViscosity +=
          particles[j].mass * (particles[j].velocity - particles[i].velocity) /
          particles[j].density * laplaceKernel(x, Constants::KERNEL_RANGE);
    }

    // Gravitational force density
    fGravity = particles[i].density * Eigen::Vector2f(0, Constants::GRAVITY);
    fPressure *= -1.0f;
    fViscosity *= Constants::VISCOCITY;

    particles[i].force += fPressure + fViscosity + fGravity;
  }
}

void SPHSolver::integrationStep(float dt) {
  for (int i = 0; i < numberParticles; i++) {
    particles[i].velocity += dt * particles[i].force / particles[i].density;
    particles[i].position += dt * particles[i].velocity;
  }
}

void SPHSolver::collisionHandling() {
  for (int i = 0; i < numberParticles; i++) {
    if (particles[i].position[0] < 0.0f) {
      particles[i].position[0] = 0.0f;
      particles[i].velocity[0] *= -0.5f;
    } else if (particles[i].position[0] > Constants::WIDTH) {
      particles[i].position[0] = Constants::WIDTH;
      particles[i].velocity[0] *= -0.5f;
    }

    if (particles[i].position[1] < 0.0f) {
      particles[i].position[1] = 0.0f;
      particles[i].velocity[1] *= -0.5f;
    } else if (particles[i].position[1] > Constants::HEIGHT) {
      particles[i].position[1] = Constants::HEIGHT;
      particles[i].velocity[1] *= -0.5f;
    }
  }
}
