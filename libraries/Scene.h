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
#include "Camera.h"

class Scene {
public:
	Scene();
	virtual ~Scene();

	void load(const char *filename);
	void bindCamera(Camera *c) { camera = c; }
	void draw(const bool *keys, Program *prog, bool isShadowPass1);
	void init();
	void update(const bool *keys, const Eigen::Vector2f &mouse, float dt);
	void checkCollisions();
	Light &getLight() { return light; }
	Vehicle &getVehicle() { return vehicle; }
	std::vector<CollisionBox> &getCollisionBoxes() { return boxes; }
	
private:
	
	std::vector<Shape> shapes; // for keeping track of world object shapes only
	std::vector<WorldObject> objects;
	std::vector<CollisionBox> boxes;
	std::vector<Texture> textures;
	Track track;
	Vehicle vehicle;
	Light light;
	Camera *camera;
	float dtCurrent;

	bool colliding[4];
	Eigen::Vector3f vnew;

	void pqpCollideWObj(WorldObject &wheel, WorldObject &wobj, int w);
	void pqpCollideTrack(WorldObject &wheel, PQP_Model *track, int w);
	void handleCollision(int w, float dist);
};

#endif
