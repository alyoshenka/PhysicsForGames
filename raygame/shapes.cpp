#include "shapes.h"
#include "physics.h"

#include "glm/glm.hpp"

#include <algorithm>
#include <iostream>

bool checkCircleCircle(glm::vec2 posA, circle circA, glm::vec2 posB, circle circB)
{
	// get the distance
	float dist = glm::length(posA - posB);

	// get the sum of the radii
	float sum = circA.radius + circB.radius;

	return dist < sum;
}

bool checkAABBAABB(glm::vec2 posA, aabb aabbA, glm::vec2 posB, aabb aabbB)
{
	// check if one axis extends across the other's

	return posA.x - aabbA.halfExtents.x < posB.x + aabbB.halfExtents.x // l r
		&& posA.x + aabbA.halfExtents.x > posB.x - aabbB.halfExtents.x // r l
		&& posA.y - aabbA.halfExtents.y < posB.y + aabbB.halfExtents.y // t b
		&& posA.y + aabbA.halfExtents.y > posB.y - aabbB.halfExtents.y; // b t
}

bool checkCircleAABB(glm::vec2 posA, circle circ, glm::vec2 posB, aabb ab)
{
	// what is point on aabb closes to circle
	// is that point inside circle

	float distX = posA.x - glm::clamp(posA.x, posB.x - ab.halfExtents.x, posB.x + ab.halfExtents.x);
	float distY = posA.y - glm::clamp(posA.y, posB.y - ab.halfExtents.y, posB.y + ab.halfExtents.y);

	return (distX * distX + distY * distY) < (circ.radius * circ.radius);
}

bool checkCirclePoint(glm::vec2 pos, float radius, glm::vec2 point)
{
	float dist = glm::length(pos - point);
	return dist < radius;
}

bool checkAABBPoint(glm::vec2 pos, aabb a, glm::vec2 point)
{
	return point.x > pos.x - a.halfExtents.x
		&& point.x < pos.x + a.halfExtents.x
		&& point.y > pos.y - a.halfExtents.y
		&& point.y < pos.y + a.halfExtents.y;
}

bool checkCircleX(glm::vec2 posA, circle lhs, glm::vec2 posB, shape rhs)
{
	return rhs.match([posA, lhs, posB] (circle s) { return checkCircleCircle(posA, lhs, posB, s); },
		             [posA, lhs, posB](aabb s)    { return checkCircleAABB(posA, lhs, posB, s); });
}

bool checkAABBX(glm::vec2 posA, aabb lhs, glm::vec2 posB, shape rhs)
{
	return rhs.match([posA, lhs, posB](circle s) { return checkCircleAABB(posA, s, posB, lhs); },
		             [posA, lhs, posB](aabb s)   { return checkAABBAABB(posA, lhs, posB, s); });
}

bool checkPointX(glm::vec2 point, glm::vec2 pos, shape rhs)
{
	return rhs.match([point, pos](circle s) { return checkCirclePoint(pos, s.radius, point); },
		[point, pos](aabb s) { return checkAABBPoint(pos, s, point); });
}

void resolvePhysBodies(physObject &lhs, physObject &rhs)
{
	lhs.collider.match(
	[&lhs, &rhs](circle circA) // a = circle
	{
		rhs.collider.match(
			[&lhs, &rhs, circA](circle circB) // b = circle
		{
			resolveCollisionCircleCircle(circA, lhs.pos, lhs.vel, lhs.mass,
				                         circB, rhs.pos, rhs.vel, rhs.mass, 1.0f);
		},
			[&lhs, &rhs, circA](aabb ab) // b = aabb
		{
			resolveCollisionCircleAABB(circA, lhs.pos, lhs.vel, 
				                       ab, rhs.pos, rhs.vel); // check order
		});
	},
	[&lhs, &rhs](aabb ab) // a = aabb
	{ 
		rhs.collider.match(
			[&lhs, &rhs, ab](circle circ) // b = circle
		{
			resolveCollisionCircleAABB(circ, rhs.pos, rhs.vel, 
				                       ab, lhs.pos, lhs.vel); // check order
		},
			[&lhs, &rhs, ab](aabb aabbB) // b = aabb
		{
			resolveCollisionAABBAABB(ab, lhs.pos, lhs.vel, 
				                     aabbB, rhs.pos, rhs.vel, 30.0f);
		});
	});
}

void resolveCollision_Old(glm::vec2 posA, glm::vec2 velA, float massA, 
	                  glm::vec2 posB, glm::vec2 velB, float massB, 
	                  float elasticity, glm::vec2 normal, glm::vec2 * dst)
{
	glm::vec2 relVel = velA - velB;
	float impulseMag = glm::dot(-(1.0f + elasticity) * relVel, normal)
		             / glm::dot(normal, normal * (1 / massA + 1 / massB));

	impulseMag /= 2.0f;

	dst[0] = velA + (impulseMag / massA) * normal;
	dst[1] = velB - (impulseMag / massB) * normal;
}

void resolveCollisionCircleCircle(circle a, glm::vec2 &posA, glm::vec2 & velA, float massA, 
	                              circle b, glm::vec2 &posB, glm::vec2 & velB, float massB, 
	                              float elasticity)
{
	glm::vec2 resImpulses[2];
	glm::vec2 normal = { 0, 0 };

	float pen = 0.0f; // penetration
	float dist = glm::length(posA - posB);
	float sum = a.radius + b.radius;

	pen = sum - dist;
	normal = glm::normalize(posA - posB);

	glm::vec2 relVel = velA - velB;
	float impulseMag = glm::dot(-(1.0f + elasticity) * relVel, normal)
		/ glm::dot(normal, normal * (1 / massA + 1 / massB));

	impulseMag /= 2.0f;

	resImpulses[0] = velA + (impulseMag / massA) * normal;
	resImpulses[1] = velB - (impulseMag / massB) * normal;

	glm::vec2 delta = normal * pen;
	posA.x += delta.x;
	posA.y += delta.y;
	posB.x -= delta.x;
	posB.y -= delta.y;

	velA = resImpulses[0];
	velB = resImpulses[1];
}

void resolveCollisionAABBAABB(aabb a, glm::vec2 &posA, glm::vec2 &velA, 
	                          aabb b, glm::vec2 &posB, glm::vec2 &velB,
	                          float timeStep)
{
	// there is a collision

	// find how far the boxes overlap

	int aIsOnLeft = posA.x <= posB.x ? 1 : -1;
	int aIsOnTop = posA.y >= posB.y ? 1 : -1;
	float eps = 0.001f;

	float xOverlap = aIsOnLeft * ((posA.x + a.halfExtents.x * aIsOnLeft) - (posB.x - b.halfExtents.x * aIsOnLeft));
	float yOverlap = aIsOnTop * ((posA.y + a.halfExtents.y * aIsOnTop) - (posB.y + b.halfExtents.y * aIsOnTop));

	bool moveAlongX = (abs(yOverlap) < eps || abs(xOverlap) < abs(yOverlap))
		&& abs(velA.x - velB.x) < eps
		&& abs(velA.y - velB.y) < eps;

	if (moveAlongX)
	{
		posA.x -= xOverlap / 2.0f;
		posB.x += xOverlap / 2.0f;
	}
	else
	{
		posA.y -= yOverlap / 2.0f;
		posB.y += yOverlap / 2.0f;
	}

	velA.x = velB.x = velA.y = velB.y = 0;

	// undo box overlap -> reset positions

	// deflect velocity, with respect to mass

	/*
	// below is swept algorithm so volumes dont skip through each other

	// https://www.gamedev.net/articles/programming/general-and-gameplay-programming/swept-aabb-collision-detection-and-response-r3084

	// find distance and time it takes to reach a collision on each axis

	// how far away the closest edges are
	float xInvEntry, yInvEntry;
	// how far away the farthest edges are
	float xInvExit, yInvExit;
	glm::vec2 normal{ 0, 0 };
	// calculate relavtive velocity
	glm::vec2 vel = velA - velB;
	vel /= timeStep;
	
	int rX = 1;
	if (posA.x > posB.x) { rX = -1; }
	vel.x *= rX;
	int rY = 1;
	if (posA.y > posB.y) { rY = -1; }
	vel.y *= rY;

	float eps = 0.001f;

	// find the distance between objects on near and far sides
	if (vel.x != 0)
	{
		xInvEntry = (posB.x - b.halfExtents.x) - (posA.x + a.halfExtents.x);
		xInvExit = (posB.x + b.halfExtents.x) - (posA.x - a.halfExtents.x);
	}
	else
	{
		xInvEntry = (posB.x + b.halfExtents.x) - (posA.x - a.halfExtents.x);
		xInvExit = (posB.x - b.halfExtents.x) - (posA.x + a.halfExtents.x);
	}

	if (vel.y != 0)
	{
		yInvEntry = (posB.y - b.halfExtents.y) - (posA.y + a.halfExtents.y);
		yInvExit = (posB.y + b.halfExtents.y) - (posA.y - a.halfExtents.y);
	}
	else
	{
		yInvEntry = (posB.y + b.halfExtents.y) - (posA.y - a.halfExtents.y);
		yInvExit = (posB.y - b.halfExtents.y) - (posA.y + a.halfExtents.y);
	}

	// find time (0-1) of collision and time (0-1) of leaving for each axis

	float xEntry, yEntry;
	float xExit, yExit;

	if (abs(vel.x) < eps)
	{
		xEntry = -std::numeric_limits<float>::infinity();
		xExit = std::numeric_limits<float>::infinity();
	}
	else
	{
		xEntry = xInvEntry / vel.x;
		xExit = xInvExit / vel.x;
	}

	if (abs(vel.y) < eps)
	{
		yEntry = -std::numeric_limits<float>::infinity();
		yExit = std::numeric_limits<float>::infinity();
	}
	else
	{
		yEntry = yInvEntry / vel.y;
		yExit = yInvExit / vel.y;
	}

	// find which axis collided first
	float entryTime = std::max(xEntry, yEntry); // time of collision
	float exitTime = std::min(xExit, yExit);

	// check to see if there was actually a collision

	// this should never happen -> there was already a collision
	if (entryTime > exitTime || xEntry * rX < 0 && yEntry * rY < 0 || xEntry > timeStep || yEntry > timeStep)
	{ 
		std::cout << entryTime << " -> " << exitTime << ", " << xEntry << " -> " << yEntry << std::endl;
		return; 
	}
	// else resolve collision
	else
	{
        // calculate normal of collided surface
        // this is easy because aabb, but will grow in complexity with other volumes

		if (xEntry > yEntry)
		{
			normal.x = xInvEntry < 0.0f ? 1.0f : -1.0f;
			normal.y = 0;
		}
		else
		{
			normal.x = 0;
			normal.y = yInvEntry < 0.0f ? 1.0f : -1.0f;
		}
	}

	// collisionTime = entryTime

	// displace

	// TO DO: displace in porportion to velocity(?)
	posA.x -= vel.x * entryTime / timeStep;
	posA.y -= vel.y * entryTime / timeStep;
	posB.x += vel.x * entryTime / timeStep;
	posB.y += vel.y * entryTime / timeStep;

	float remainingTime = 1.0f - entryTime;
	float magnitude = sqrt(vel.x * vel.x + vel.y * vel.y) * remainingTime;
	float dotProd = vel.x * normal.y + velA.y * normal.x;

	// direct object in new direction

	enum Response { deflect, push, slide };
	Response desiredResponse = deflect;

	switch (desiredResponse)
	{
	case deflect:
		// reduce velocity by remaining time
		velA.x *= remainingTime / timeStep;
		velB.y *= remainingTime / timeStep;
		velB.x *= remainingTime / timeStep;
		velB.y *= remainingTime / timeStep;

		// negate velocity on whichever axis had collision
		if (abs(normal.x) > eps) 		
		{ 
			velA.x *= -1; 
			velB.x *= -1;
		}
		if (abs(normal.y) > eps) 
		{ 
			velA.y *= -1; 
			velB.y *= -1;
		}
		break;
	case push:
		// reuses remaining velocity and pushes in direction parallel to edge
		if (dotProd > 0.0f) { dotProd = 1; }
		else if (dotProd < 0.0f) { dotProd = -1; }
		velA.x = dotProd * normal.y * magnitude;
		velA.y = dotProd * normal.x * magnitude;
		break;
	case slide:
		// doesn't go as fast as push
		velA.x = dotProd * normal.y;
		velA.y = dotProd * normal.x;
		break;
	default:
		assert(false, "invalid collision response");
		break;
	}
	*/
}

void resolveCollisionCircleAABB(circle c, glm::vec2 &posA, glm::vec2 &velA,
	                            aabb ab, glm::vec2 &posB, glm::vec2 &velB)
{
	// what is point on aabb closes to circle
	// is that point inside circle

	float distX = posA.x - glm::clamp(posA.x, posB.x - ab.halfExtents.x, posB.x + ab.halfExtents.x);
	float distY = posA.y - glm::clamp(posA.y, posB.y - ab.halfExtents.y, posB.y + ab.halfExtents.y);

	int cOnLeft = posA.x <= posB.x ? 1 : -1;
	int cOnTop = posA.y <= posB.y ? 1 : -1;

	posA.x -= distX / 2.0f * cOnLeft;
	posB.x += distX / 2.0f * cOnLeft;
	posA.y -= distY / 2.0f * cOnTop;
	posB.y += distY / 2.0f * cOnTop;

	float eps = 0.001f;
	if (abs(distX) < eps || abs(distY) < eps)
	{
		std::cout << "a: " << posA.x << ", " << posA.y << std::endl;
		std::cout << "b: " << posB.x << ", " << posB.y << std::endl;
	}

	velA.x = velA.y = velB.x = velB.y = 0;
}
