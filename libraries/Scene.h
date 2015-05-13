#pragma  once
#ifndef __Scene__
#define __Scene__

#include <vector>
#include <Eigen/Dense>
#include "Shape.h"
#include "Track.h"
#include "WorldObject.h"
#include "Texture.h"
#include "MatrixStack.h"
#include "Program.h"
#include "CollisionBox.h"
#include "Vehicle.h"
#include "../libraries/Light.h"
#include "PQP.h"

class Scene {
public:
	virtual ~Scene();

	void load(const char *filename);
	void draw(const bool *keys, MatrixStack &MV, MatrixStack &P, Program *prog, Light &light, bool isShadowPass1);
	void init();
	void update(const bool *keys, const Eigen::Vector2f &mouse, float dt);
	void checkCollisions();
	Vehicle &getVehicle() { return vehicle; }
	std::vector<CollisionBox> &getCollisionBoxes() { return boxes; }
	
private:
	
	std::vector<Shape> shapes; // for keeping track of world object shapes only
	std::vector<WorldObject> objects;
	std::vector<CollisionBox> boxes;
	std::vector<Texture> textures;
	std::vector<int> shapespertex;
	Track track;
	Vehicle vehicle;

	void pqpCollideWObj(WorldObject &wheel, WorldObject &wobj);
	void pqpCollideTrack(WorldObject &wheel, PQP_Model *track);
};

#endif
