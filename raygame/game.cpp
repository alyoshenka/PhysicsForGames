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

	clickReset = 0.1f;
	clickElapsed = clickReset;
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


	if ((mb0 || mb1) && clickElapsed > clickReset)
	{
		clickElapsed = 0.0f;

		physObject babyPhys = physObject(false, physObjects.size() + 1);
		auto mousePos = GetMousePosition();
		babyPhys.pos = { mousePos.x, mousePos.y };
		babyPhys.addForce({ 0, 5000 });
		babyPhys.name = std::to_string(physObjects.size() + 1);
		babyPhys.collider = circle({ 20 });
		physObjects.push_back(babyPhys);
		std::cout << "Added physics object " << babyPhys.name << std::endl;

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

	checkCollisions(physObjects);
	resolveCollisions(physObjects);

	for (physObject ob : physObjects)
	{
		ob.swapCollisionLists();

		assert(ob.collidingObjects->size() == 0);
	}
}

void game::draw() const
{
	// Draw
	//----------------------------------------------------------------------------------
	BeginDrawing();

	ClearBackground(RAYWHITE);

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
