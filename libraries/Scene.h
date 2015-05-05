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

class Scene {
public:
	virtual ~Scene();

	void load(const char *filename);
	void draw(const bool *keys, MatrixStack &MV, Program *prog);
	void init();
	void update(const bool *keys, const Eigen::Vector2f &mouse, float dt);
	Vehicle &getVehicle() { return vehicle; }
	std::vector<CollisionBox> &getCollisionBoxes() { return boxes; }
	
private:
	
	std::vector<Shape> shapes;
	std::vector<WorldObject> objects;
	std::vector<CollisionBox> boxes;
	std::vector<Texture> textures;
	std::vector<int> shapespertex;
	Track track;
	Vehicle vehicle;
};

#endif
