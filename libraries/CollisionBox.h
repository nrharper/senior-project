#pragma once
#ifndef __Collision__
#define __Collision__

#define GLM_FORCE_RADIANS
#include "glm/glm.hpp"

class CollisionBox
{
public:
	void setBounds(float, float, float, float, float, float);
   bool isXZCollision(const CollisionBox &box);
   float getMinX() const { return minX; }
   float getMinY() const { return minY; }
   float getMinZ() const { return minZ; }
   float getMaxX() const { return maxX; }
   float getMaxY() const { return maxY; }
   float getMaxZ() const { return maxZ; }
	
private:
   float minX, maxX, minY, maxY, minZ, maxZ;
};

#endif
