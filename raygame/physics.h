#pragma once

#include "glm/vec2.hpp"
#include "shapes.h"

#include <vector>

struct collision;
class collisions;

class physObject
{
	// update the physics forces on the object
	void updateForces(float delta);

public:
	glm::vec2 pos;
	glm::vec2 vel;

	glm::vec2 forces;

	physObject();

	float mass;
	float drag;

	shape collider;

	void tickPhys(float delta);

	void drawCollider() const;
	void draw() const;

	// add continuous force with respect to mass
	void addForce(glm::vec2 force);
	// add instantaneous force with respect to mass
	void addImpulse(glm::vec2 impulse);
	// accelerates the object without respect to mass
	void addAcceleration(glm::vec2 accel);
	// adds an instantateous force without respect to mass
	void addVelocityChange(glm::vec2 delta);

	void onCollisionEnter(physObject collision);
	void onCollisionStay(physObject collision);
	void onCollisionExit(physObject collision);
};

class collisions
{
	std::vector<collision>* prevCollisions;
	
	void dealWithCollisions();

public:

	collisions();

	std::vector<collision>* currCollisions;

	void checkCollisions(std::vector<physObject>& objects);
};

struct collision
{
	physObject a;
	physObject b;
};