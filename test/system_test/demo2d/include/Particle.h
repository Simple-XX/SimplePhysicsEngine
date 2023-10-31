#pragma once

#include <Eigen/Dense>

class Particle
{
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