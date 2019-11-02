#pragma once

#include "glm/vec2.hpp"
#include "mapbox/variant.hpp"

struct circle
{
	float radius;
	// glm::vec2 pos;
};

struct aabb
{
	glm::vec2 halfExtents;
	// glm::vec2 pos;
};

struct point
{
	glm::vec2 pos;
};

// c++ 11 "typedef"
using shape = mapbox::util::variant<circle, aabb>;

bool checkCircleCircle(glm::vec2 posA, circle circA, glm::vec2 posB, circle circB);
bool checkAABBAABB(glm::vec2 posA, aabb aabbA, glm::vec2 posB, aabb aabbB);
bool checkCircleAABB(glm::vec2 posA, circle circ, glm::vec2 posB, aabb ab);
bool checkCirclePoint(glm::vec2 pos, float radius, glm::vec2 point);
bool checkAABBPoint(glm::vec2 pos, aabb a, glm::vec2 point);

bool checkCircleX(glm::vec2 posA, circle lhs, glm::vec2 posB, shape rhs);
bool checkAABBX(glm::vec2 posA, aabb lhs, glm::vec2 posB, shape rhs);
bool checkPointX(glm::vec2 point, glm::vec2 pos, shape rhs);

void resolvePhysBodies(class physObject& lhs, class physObject& rhs);
void resolveCollision(glm::vec2 posA, glm::vec2 velA, float massA,
	                  glm::vec2 posB, glm::vec2 velB, float massB,
	                  float elasticity, glm::vec2 normal, glm::vec2* dst);