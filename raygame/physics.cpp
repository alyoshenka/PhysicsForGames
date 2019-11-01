#include "physics.h"

#include "raylib.h"

#include <iostream>

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
	drag = 1.0f;

	name = "none";

	isTrigger = false;
	id = -1;

	collidingObjects = new std::vector<physObject*>();
	prevCollidingObjects = new std::vector<physObject*>();
}

physObject::physObject(bool setAsTrigger, int initID) : physObject()
{
	isTrigger = setAsTrigger;
	id = initID;
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

#pragma region ForceManipulation

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

#pragma endregion ForceManipulation

#pragma region InteractionEvents

void physObject::onCollisionEnter(physObject collision)
{
	std::cout << "collision enter on " << name << " by " << collision.name << std::endl;
}

void physObject::onCollisionStay(physObject collision)
{
	std::cout << "collision stay on " << name << " by " << collision.name << std::endl;
}

void physObject::onCollisionExit(physObject collision)
{
	std::cout << "collision exit on " << name << " by " << collision.name << std::endl;
}

void physObject::onTriggerEnter(physObject trigger)
{
	std::cout << "trigger enter on " << name << " by " << trigger.name << std::endl;
}

void physObject::onTriggerStay(physObject trigger)
{
	std::cout << "trigger stay on " << name << " by " << trigger.name << std::endl;
}

void physObject::onTriggerExit(physObject trigger)
{
	std::cout << "trigger exit on " << name << " by " << trigger.name << std::endl;
}

#pragma endregion InteractionEvents

bool physObject::getIsTrigger()
{
	return isTrigger;
}

void physObject::swapCollisionLists()
{
	prevCollidingObjects->clear();
	for (int i = 0; i < collidingObjects->size(); i++)
	{
		prevCollidingObjects->push_back(collidingObjects->at(i));
	}
	collidingObjects->clear();
}

int physObject::getID()
{
	return id;
}

bool physObject::operator==(physObject & rhs) const
{
	return id == rhs.getID();
}

void checkCollisions(std::vector<physObject>& objects)
{
	physObject *iObj;
	physObject *jObj;

	// add all new collisions to currCollisions
	for (int i = 0; i < objects.size(); i++)
	{
		for (int j = 0; j < objects.size(); j++)
		{
			iObj = &(objects.at(i));
			jObj = &(objects.at(j));

			if (iObj->name.compare(jObj->name) == 0) { continue; } // skip self

			iObj->collider.match(
				[iObj, jObj](circle c)
			{
				if (checkCircleX(iObj->pos, c, jObj->pos, jObj->collider))
				{
					iObj->collidingObjects->push_back(jObj);
					jObj->collidingObjects->push_back(iObj);
				}
			},
				[iObj, jObj](aabb c)
			{
				if (checkAABBX(iObj->pos, c, jObj->pos, jObj->collider))
				{
					iObj->collidingObjects->push_back(jObj);
					jObj->collidingObjects->push_back(iObj);
				}
			});
		}
	}
}

void resolveCollisions(std::vector<physObject>& objects)
{
	// ontriggerenter/exit only gets called when the object
	// starts/stops colliding with *anything*, not just any
	// one object

	// the reason this isn't working is that i am checking for collision
	// with anything, not just the object

	// to solve I could have each object have a list of collisions and update it 
	// every frame

	// if collision in newCol and not oldCol: onCollisionEnter
	// if collision in oldCol and newCol: onCollisionStay
	// if collision in oldCol and not newCol: onCollisionExit

	std::vector<collision> collisions;

	// sort collisions
	physObject object;
	for (int i = 0; i < objects.size(); i++)
	{
		object = objects.at(i);

		// sort collisions

		// stay, exit
		for (int j = 0; j < object.prevCollidingObjects->size(); j++)
		{
			physObject *jObj = object.prevCollidingObjects->at(j);
			physObject *kObj = nullptr;
			bool prevInCur = false;
			for (int k = 0; k < object.collidingObjects->size(); k++)
			{
				kObj = object.collidingObjects->at(k);
				if (jObj == kObj) 
				{
					prevInCur = true;
					assert(nullptr != kObj);
					// object.prevCollidingObjects->erase(object.prevCollidingObjects->begin() + k);
					break;
				}
			}
			collisions.push_back(collision(prevInCur ? stay : exitStage, jObj, kObj));
		}

		// enter
		for (int j = 0; j < object.collidingObjects->size(); j++)
		{
			physObject *jObj = object.collidingObjects->at(j);
			physObject *kObj = nullptr;
			bool curInPrev = false;
			for (int k = 0; k < object.prevCollidingObjects->size(); k++)
			{
				kObj = object.prevCollidingObjects->at(k);
				if (jObj == kObj)
				{
					curInPrev = true;
					break;
				}
			}
			if (!curInPrev && nullptr != kObj);
			{
				
				collisions.push_back(collision(enter, jObj, kObj));
				std::cout << collisions.at(collisions.size() - 1).object->name << std::endl;
			}
		}
	}	

	for (collision c : collisions)
	{
		std::cout << c.subject->name << ", " << c.object->name << std::endl;
		return;
		bool isTrigger = c.subject->getIsTrigger() || c.object->getIsTrigger();
		switch (c.stage)
		{
		case enter:	
			if (isTrigger) { c.subject->onTriggerEnter(*c.object); }
			else { c.subject->onCollisionEnter(*c.object); }
			break;
		case stay:
			if (isTrigger) { c.subject->onTriggerStay(*c.object); }
			else { c.subject->onCollisionStay(*c.object); }
			break;
		case exitStage:
			if (isTrigger) { c.subject->onTriggerExit(*c.object); }
			else { c.subject->onCollisionExit(*c.object); }
			break;
		default:
			assert(false);
			break;
		}
	}
}

collision::collision(Flag _t, physObject * _s, physObject * _o)
{
	stage = _t;
	subject = _s;
	object = _o;
}
