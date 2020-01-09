#include "game.h"
#include "raylib.h"

#include "mapbox/variant.hpp"

#include <iostream>
#include <cmath>

game::game()
{
	targetFixedStep = 1.0f / 30.0f; // target physics tick rate
	accumulatedFixedTime = 0.0f;

	screenWidth = 800;
	screenHeight = 450;

	clickReset = 0.3f;
	clickElapsed = clickReset;

	currentPointerObject = nullptr;

	vectorAngle = 0;
	vectorMagnitude = 500;
}

void game::init()
{
	// Initialization
	//--------------------------------------------------------------------------------------
	InitWindow(screenWidth, screenHeight, "raylib [core] example - basic window");

	SetTargetFPS(60);
	//--------------------------------------------------------------------------------------
}

void game::tick()
{
	accumulatedFixedTime += GetFrameTime();
	clickElapsed += GetFrameTime();

	bool mb0 = IsMouseButtonDown(0);
	bool mb1 = IsMouseButtonDown(1);
	bool one = IsKeyPressed(KEY_ONE);
	bool two = IsKeyPressed(KEY_TWO);
	bool three = IsKeyPressed(KEY_THREE);
	bool four = IsKeyPressed(KEY_FOUR);
	int scrollY = GetMouseWheelMove();

	auto mousePos = GetMousePosition();
	this->mousePos = { mousePos.x, mousePos.y };

	if ((mb0 || mb1) && clickElapsed > clickReset)
	{
		clickElapsed = 0.0f;

		physObject babyPhys;
		babyPhys.pos = { mousePos.x, mousePos.y };
		babyPhys.name = std::to_string(physObjects.size() + 1);
		if (mb0) { babyPhys.collider = circle{ 20.0f }; }
		else { babyPhys.collider = aabb{ {15.0f, 15.0f} }; }
		physObjects.push_back(babyPhys);
		std::cout << "Added physics object " << babyPhys.name << std::endl;
	}

	glm::vec2 calculatedVecAngle{ cos(vectorAngle) * vectorMagnitude, -sin(vectorAngle) * vectorMagnitude };
	if (one) { currentPointerObject->addForce(calculatedVecAngle); }
	if (two) { currentPointerObject->addImpulse(calculatedVecAngle); }
	if (three) { currentPointerObject->addAcceleration(calculatedVecAngle); }
	if (four) { currentPointerObject->addVelocityChange(calculatedVecAngle); }

	if (scrollY > 0) { vectorAngle += 0.1f; }
	else if (scrollY < 0) { vectorAngle -= 0.1f; }
	if (vectorAngle > 2 * PI) { vectorAngle -= (2 * PI); }
	if (vectorAngle < 0) { vectorAngle += (2 * PI); }
}

void game::tickPhys()
{
	accumulatedFixedTime -= targetFixedStep;

	for (auto& i : physObjects)
	{
		i.tickPhys(targetFixedStep);
	}

	for (auto& i : physObjects)
	{
		for (auto &j : physObjects)
		{
			if (&i == &j) { continue; }

			bool collision = false;

			i.collider.match(
				[i, j, &collision](circle c) 
			{ 
				collision = checkCircleX(i.pos, c, j.pos, j.collider);
			},
				[i, j, &collision](aabb a) 
			{
				collision = checkAABBX(i.pos, a, j.pos, j.collider);
			});

			if (collision) { resolvePhysBodies(i, j); }
		}

		if (checkPointX(mousePos, i.pos, i.collider)) 
		{
			i.color = RED;
			currentPointerObject = &i;
		}
		else { i.color = BLUE; }
	}
}

void game::draw() const
{
	// Draw
	//----------------------------------------------------------------------------------
	BeginDrawing();

	ClearBackground(RAYWHITE);

	DrawText("Click mouse to add object", 10, 10, 20, GRAY);
	DrawText("Hover over object to select", 10, 30, 20, GRAY);
	DrawText("Press number buttons to interact", 10, 50, 20, GRAY);
	DrawText("1: Add Force", 10, 70, 20, GRAY);
	DrawText("2: Add Impulse", 10, 90, 20, GRAY);
	DrawText("3: Add Acceleration", 10, 110, 20, GRAY);
	DrawText("4: Add VelocityChange", 10, 130, 20, GRAY);
	DrawText("Scroll mouse wheel to change direction", 10, 150, 20, GRAY);
	std::string s = "Direction: " + std::to_string(vectorAngle * RAD2DEG);
	DrawText(s.c_str(), 10, 170, 20, GRAY);

	for (const physObject i : physObjects)
	{
		i.draw();
	}

	EndDrawing();
	//----------------------------------------------------------------------------------
}

void game::exit()
{
	// De-Initialization
	//--------------------------------------------------------------------------------------   
	CloseWindow();        // Close window and OpenGL context
	//--------------------------------------------------------------------------------------

}

void game::screenWrap()
{
	for (auto& i : physObjects)
	{
		if (i.pos.x > screenWidth) { i.pos.x = 0; }
		if (i.pos.x < 0.0f) { i.pos.x = screenWidth; }
		if (i.pos.y > screenHeight) { i.pos.y = 0; }
		if (i.pos.y < 0.0f) { i.pos.y = screenHeight; }
	}
}

bool game::shouldClose() const
{
	return WindowShouldClose();
}

bool game::shouldPhysics() const
{
	return accumulatedFixedTime >= targetFixedStep;
}
