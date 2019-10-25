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

// c++ 11 "typedef"
using shape = mapbox::util::variant<circle, aabb>;

bool checkCircleCircle(glm::vec2 posA, circle circA, glm::vec2 posB, circle circB);
bool checkAABBAABB(glm::vec2 posA, aabb aabbA, glm::vec2 posB, aabb aabbB);
bool checkCircleAABB(glm::vec2 posA, circle circ, glm::vec2 posB, aabb ab);

bool checkCircleX(glm::vec2 posA, circle lhs, glm::vec2 posB, shape rhs);
bool checkAABBX(glm::vec2 posA, aabb lhs, glm::vec2 posB, shape rhs);