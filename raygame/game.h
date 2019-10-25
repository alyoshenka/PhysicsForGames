#pragma once

#include <vector>

#include "physics.h"

class game
{
	float accumulatedFixedTime;

	std::vector<physObject> physObjects;
	collisions collisionData;

public:
	game();

	void init();
	void tick();
	void tickPhys();
	void draw() const; // const so it doesn't modify state
	void exit();

	bool shouldClose() const;
	bool shouldPhysics() const;

	float targetFixedStep; // time between fixed updates
};
