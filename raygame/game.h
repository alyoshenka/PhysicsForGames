#pragma once

#include <vector>

#include "physics.h"

class game
{
	float accumulatedFixedTime;

	int screenWidth;
	int screenHeight;

	float clickReset;
	float clickElapsed;

	std::vector<physObject> physObjects;

public:
	float vectorAngle; // degrees
	float vectorMagnitude;
	float mouseScroll;
	int recSize;
	glm::vec2 mousePos;
	physObject *currentPointerObject;

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
