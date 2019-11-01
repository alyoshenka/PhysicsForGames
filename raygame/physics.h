#pragma once

#include "glm/vec2.hpp"
#include "shapes.h"

#include <vector>
#include <string>

class collisionHandler;

struct collision;

enum Flag
{
	undetermined,
	enter,
	stay,
	exitStage
};

class physObject
{
	// bool isTrigger;
	// int id;

	// update the physics forces on the object
	void updateForces(float delta);

public:
	glm::vec2 pos;
	glm::vec2 vel;

	glm::vec2 forces;

	physObject();
	// physObject(bool setAsTrigger, int initID);

	float mass;
	float drag;

	shape collider;
	std::string name;
	std::vector<physObject*> *prevCollidingObjects;
	std::vector<physObject*> *collidingObjects;

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

	/*
	void onCollisionEnter(physObject collision);
	void onCollisionStay(physObject collision);
	void onCollisionExit(physObject collision);

	void onTriggerEnter(physObject trigger);
	void onTriggerStay(physObject trigger);
	void onTriggerExit(physObject trigger);

	bool getIsTrigger(); // returns isTrigger
	// swaps current collision list into previous collision list
	void swapCollisionLists();
	int getID();

	bool operator ==(physObject &rhs) const;
	*/
};

/*
// collision only works one way
class collision
{
public:
	Flag stage;
	physObject *subject;
	physObject *object;
	collision(Flag _t, physObject *_s, physObject *_o);
};
*/

// void checkCollisions(std::vector<physObject>& objects);

// void resolveCollisions(std::vector<physObject>& objects);

