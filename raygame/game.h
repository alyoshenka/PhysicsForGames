#pragma once

#include <vector>

#include "physics.h"

class game
{
	float accumulatedFixedTime;

	int screenWidth;
	int screenHeight;

	std::vector<physObject> physObjects;
	collisions collisionData;

public:
	game();

	void init();
	void tick();
	void tickPhys();
	void draw() const; // const so it doesn't modify state
	void exit();
	void screenWrap();

	bool shouldClose() const;
	bool shouldPhysics() const;

	float targetFixedStep; // time between fixed updates
};
