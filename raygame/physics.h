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

	void addForce(glm::vec2 force);
};