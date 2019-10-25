#pragma once

#include "glm/vec2.hpp"

class physObject
{
public:
	glm::vec2 pos;
	glm::vec2 vel;

	glm::vec2 forces;

	physObject();

	float mass;
	float drag;

	void tickPhys(float delta);
	void draw() const;

	// add continuous force with respect to mass
	void addForce(glm::vec2 force);
	// add instantaneous force with respect to mass
	void addImpulse(glm::vec2 impulse);
	// accelerates the object without respect to mass
	void addAcceleration(glm::vec2 accel);
	// adds an instantateous force without respect to mass
	void addVelocityChange(glm::vec2 delta);
};