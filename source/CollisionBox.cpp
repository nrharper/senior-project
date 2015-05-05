#include "../libraries/CollisionBox.h"

using namespace std;

void CollisionBox::setBounds(float minX, float maxX, float minY, float maxY, float minZ, float maxZ) {
	this->minX = minX;
	this->maxX = maxX;
	this->minY = minY;
	this->maxY = maxY;
	this->minZ = minZ;
	this->maxZ = maxZ;
}

bool CollisionBox::isXZCollision(const CollisionBox &box) {
   return !(maxX < box.getMinX() || box.getMaxX() < minX || maxZ < box.getMinZ() || box.getMaxZ() < minZ);
}
