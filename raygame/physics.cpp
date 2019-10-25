#include "physics.h"

#include "raylib.h"

void physObject::updateForces(float delta)
{
	// integrate forces into velocity
	vel += forces * delta;
	forces = { 0, 0 };

	// integrate linear drag
	vel *= 1.0f - delta * drag;

	// integrate velocity into position
	pos += vel * delta;
}

physObject::physObject()
{
	pos = { 0, 0 };
	vel = { 0, 0 };
	forces = { 0, 0 };

	mass = 1.0f;
	drag = 0.5f;
}

void physObject::tickPhys(float delta)
{
	updateForces(delta);
}

void physObject::drawCollider() const
{
	collider.match([this](circle c) { if (checkCircleX(pos, c, pos, collider)) { DrawCircleLines(pos.x, pos.y, c.radius, BLACK); } },
		           [this](aabb c)   { if (checkAABBX(pos, c, pos, collider))   { DrawRectangleLines(pos.x - c.halfExtents.x, pos.y - c.halfExtents.y, c.halfExtents.x * 2, c.halfExtents.y * 2, BLACK); } });
}

void physObject::draw() const
{
	drawCollider();
}

void physObject::addForce(glm::vec2 force)
{
	forces += force * (1.0f / mass);
}

void physObject::addImpulse(glm::vec2 impulse)
{
	vel += impulse * (1.0f / mass);
}

void physObject::addAcceleration(glm::vec2 accel)
{
	forces += accel;
}

void physObject::addVelocityChange(glm::vec2 delta)
{
	vel += delta;
}

#include <iostream>

// new collisions should be added to a separate array, then dealt with

void collisions::dealWithCollisions()
{
	// if collision in newCol and not oldCol: onCollisionEnter
	// if collision in oldCol and newCol: onCollisionStay
	// if collision in oldCol and not newCol: onCollisionExit

	// THIS IS NOT PERFORMANT
	std::vector<int> newCollisions;
	std::vector<int> aveCollisions;
	std::vector<int> oldCollisions;

	// check for collisions that stayed or are new
	for (int i = 0; i < prevCollisions->size(); i++)
	{
		bool newInOld = false;
		for (int j = 0; j < currCollisions->size(); j++) 
		{			
			if (&(prevCollisions[i]) == &(currCollisions[j]))
			{
				aveCollisions.push_back(i);
				newInOld = true;
			}
		}

		if (!newInOld)
		{
			newCollisions.push_back(i);
		}
	}

	// check for collisions that are old
	for (int i = 0; i < currCollisions->size(); i++)
	{
		bool oldInNew = false;
		for (int j = 0; j < prevCollisions->size(); j++)
		{
			if (&(prevCollisions[i]) == &(currCollisions[j]))
			{
				oldInNew = true;
			}
		}

		if (!oldInNew)
		{
			oldCollisions.push_back(i);
		}
	}

	// debug collisions
	// old
	for (int i = 0; i < oldCollisions.size(); i++)
	{
		std::cout << "collision exit" << std::endl;
	}

	// new
	for (int i = 0; i < newCollisions.size(); i++)
	{
		std::cout << "collision enter" << std::endl;
	}
}

collisions::collisions()
{
	prevCollisions = new std::vector<collision>();
	currCollisions = new std::vector<collision>();
}

void collisions::checkCollisions(std::vector<physObject>& objects)
{
	std::vector<collision>* hold;
	hold = prevCollisions;
	prevCollisions = currCollisions;
	currCollisions = hold;
	currCollisions->clear();

	// add all new collisions to currCollisions
	for (auto& i : objects)
	{
		for (auto& j : objects)
		{
			if (&i == &j) { continue; } // skip self

			bool isCollision = false;

			i.collider.match(
			[i, j, this, &isCollision](circle c)
			{ 
				if (checkCircleX(i.pos, c, j.pos, j.collider))
				{ 
					// currCollisions->push_back({ i, j });
					std::cout << "collision" << std::endl;
					isCollision = true;
				} 
			},
			[i, j, this, &isCollision](aabb c) 
			{ 
				if (checkAABBX(i.pos, c, j.pos, j.collider)) 
				{ 
					// currCollisions->push_back({ i, j });
					std::cout << "collision" << std::endl;
					isCollision = true;
				} 
			});

			if (isCollision) { resolvePhysBodies(i, j); }
		}
	}

	dealWithCollisions();
}
