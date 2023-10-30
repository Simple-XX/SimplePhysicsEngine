#pragma once

#include <vector>

#include "Particle.h"
#include "Grid.h"

enum class Visualization
{
	Default,
	Velocity,
	Force,
	Density,
	Pressure,
	Water
};

class SPHSolver
{
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

