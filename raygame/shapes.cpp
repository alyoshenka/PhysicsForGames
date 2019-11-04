#include "shapes.h"
#include "physics.h"

#include "glm/glm.hpp"

#include <algorithm>

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

void resolvePhysBodies(physObject & lhs, physObject & rhs)
{
	lhs.collider.match(
		[lhs, rhs](circle a)
	{
		rhs.collider.match(
			[lhs, rhs, a](circle b)
		{
			glm::vec2 resImpulses[2];
			glm::vec2 normal = { 0, 0 };

			float pen = 0.0f; // penetration
			float dist = glm::length(lhs.pos - rhs.pos);
			float sum = a.radius + b.radius;

			pen = sum - dist;
			normal = glm::normalize(lhs.pos - rhs.pos);

			resolveCollisionCircleCircle(lhs.pos, lhs.vel, lhs.mass,
				rhs.pos, rhs.vel, rhs.mass,
				1.0f, normal, resImpulses);

			glm::vec2 delta = normal * pen;
			lhs.pos.x += delta.x;
			lhs.pos.y += delta.y;
			rhs.pos.x -= delta.x;
			rhs.pos.y -= delta.y;

			lhs.vel = resImpulses[0];
			rhs.vel = resImpulses[1];
		},
			[lhs, rhs](aabb b)
		{
			assert(false && "not yet implemented");
		});
	},
		[lhs, rhs](aabb a) 
	{ 
		assert(false && "not yet implemented");
	});
}

void resolveCollisionCircleCircle(glm::vec2 posA, glm::vec2 velA, float massA, 
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

// THIS ASSUMES B IS NOT MOVING
void resolveCollisionAABBAABB(aabb a, glm::vec2 posA, glm::vec2 &velA, aabb b, glm::vec2 posB, glm::vec2 &velB, float & normalx, float & normaly)
{
	// https://www.gamedev.net/articles/programming/general-and-gameplay-programming/swept-aabb-collision-detection-and-response-r3084

	// find distance and time it takes to reach a collision on each axis

	// how far away the closest edges are
	float xInvEntry, yInvEntry;
	// how far away the farthest edges are
	float xInvExit, yInvExit;

	// find the distance between objects on near and far sides
	if (velA.x > 0)
	{
		xInvEntry = posB.x - (posA.x + a.halfExtents.x);
		xInvExit = (posB.x + b.halfExtents.x) - posA.x;
	}
	else
	{
		xInvEntry = (posB.x + b.halfExtents.x) - posA.x;
		xInvExit = posB.x - (posA.x + a.halfExtents.x);
	}

	if (velA.y > 0)
	{
		yInvEntry = posB.y - (posA.y + a.halfExtents.y);
		yInvExit = (posB.y + b.halfExtents.y) - posA.y;
	}
	else
	{
		yInvEntry = (posB.y + b.halfExtents.y) - posA.y;
		yInvExit = posB.y - (posA.y + a.halfExtents.y);
	}

	// find time (0-1) of collision and time (0-1) of leaving for each axis

	float xEntry, yEntry;
	float xExit, yExit;

	if (velA.x == 0.0f)
	{
		xEntry = -std::numeric_limits<float>::infinity();
		xExit = std::numeric_limits<float>::infinity();
	}
	else
	{
		xEntry = xInvEntry / velA.x;
		xExit = xInvExit / velA.x;
	}

	if (velB.y == 0.0f)
	{
		yEntry = -std::numeric_limits<float>::infinity();
		yExit = std::numeric_limits<float>::infinity();
	}
	else
	{
		yEntry = yInvEntry / velA.y;
		yExit = yInvExit / velA.y;
	}

	// find which axis collided first
	float entryTime = std::max(xEntry, yEntry); // time of collision
	float exitTime = std::min(xEntry, yEntry);

	// check to see if there was actually a collision
	if (entryTime > exitTime || xEntry < 0 && yEntry < 0 || xEntry > 1 || yEntry > 1)
	{
		return;
	}

	// else resolve collision

	// calculate normal of collided surface
	float normalX, normalY;

	// this is easy because aabb, but will grow in complexity with other volumes

	if (xEntry > yEntry)
	{
		normalX = xInvEntry < 0 ? 1 : -1;
		normalY = 0;
	}
	else
	{
		normalX = 0;
		normalY = yInvEntry < 0 ? 1 : -1;
	}

	posA.x += velA.x * entryTime;
	posA.y += velA.y * entryTime;

	float remainingTime = 1.0f - entryTime;

	// direct object in new direction

	enum Response { deflect, push, slide };
	Response desiredResponse = deflect;

	switch (desiredResponse)
	{
	case deflect:
		// reduce velocity by remaining time
		velA.x *= remainingTime;
		velB.y *= remainingTime;
		// negate velocity on whichever axis had collision
		if (abs(normalX) > 0.0001f) { velA.x *= -1; }
		if (abs(normalY) > 0.0001f) { velA.y *= -1; }
		break;
	case push:
		// reuses remaining velocity and pushes in direction parallel to edge
		float magnitude = sqrt(velA.x * velA.x + velA.y * velA.y) * remainingTime;
		float dotProd = velA.x * normalY + velA.y * normalX;
		if (dotProd > 0.0f) { dotProd = 1; }
		else if (dotProd < 0.0f) { dotProd = -1; }
		velA.x = dotProd * normalY * magnitude;
		velA.y = dotProd * normalX * magnitude;
		break;
	case slide:
		// doesn't go as fast as push
		float magnitude = sqrt(velA.x * velA.x + velA.y * velA.y) * remainingTime;
		velA.x = dotProd * normalY;
		velA.y = dotProd * normalX;
		break;
	default:
		assert(false, "invalid collision response");
		break;
	}
}
