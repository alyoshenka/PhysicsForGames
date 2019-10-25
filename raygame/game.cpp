#include "game.h"
#include "raylib.h"

#include "mapbox/variant.hpp"

#include <iostream>

game::game()
{
	targetFixedStep = 1.0f / 30.0f; // target physics tick rate
	accumulatedFixedTime = 0.0f;

	screenWidth = 800;
	screenHeight = 450;
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

	bool mb0 = IsMouseButtonPressed(0);
	bool mb1 = IsMouseButtonPressed(1);


	if (mb0 || mb1)
	{
		physObjects.emplace_back();
		std::cout << "Added a physics object" << std::endl;

		auto& babyPhys = physObjects[physObjects.size() - 1];
		auto mousePos = GetMousePosition();
		babyPhys.pos = { mousePos.x, mousePos.y };
		// babyPhys.addForce({ 0, 3000 });

		if (mb0) 
		{ 
			babyPhys.collider = circle{ 20.0f };
		}
		else 
		{ 
			babyPhys.collider = aabb{ {15.0f, 15.0f} }; 
		}
	}
}

void game::tickPhys()
{
	accumulatedFixedTime -= targetFixedStep;

	for (auto& i : physObjects)
	{
		i.tickPhys(targetFixedStep);
	}

	collisionData.checkCollisions(physObjects);
}

void game::draw() const
{
	// Draw
	//----------------------------------------------------------------------------------
	BeginDrawing();

	ClearBackground(RAYWHITE);

	for (const auto& i : physObjects)
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
		if (i.pos.x < 0) { i.pos.x = screenWidth; }
		if (i.pos.y > screenHeight) { i.pos.y = 0; }
		if (i.pos.y < 0) { i.pos.y = screenHeight; }
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
