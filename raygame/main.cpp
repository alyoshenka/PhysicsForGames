/*******************************************************************************************
*
*   raylib [core] example - basic window
*
*   This example has been created using raylib 1.0 (www.raylib.com)
*   raylib is licensed under an unmodified zlib/libpng license (View raylib.h for details)
*
*   Copyright (c) 2013-2016 Ramon Santamaria (@raysan5)
*
********************************************************************************************/

#include "game.h"
#include "raylib.h"

int main()
{	
	game app;
	app.init();
	app.targetFixedStep = 1.0f / 30.0f; // target physics tick rate

	while (!app.shouldClose())
	{
		app.tick();

		app.screenWrap();

		while (app.shouldPhysics())
		{
			app.tickPhys();
		}

		app.draw();
	}

	app.exit();
	
	return 0;
}