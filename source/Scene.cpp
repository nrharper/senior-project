#include "../libraries/Scene.h"
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include "PQP.h"

using namespace std;

Scene::Scene()
{
}

Scene::~Scene()
{
	
}

void Scene::draw(const bool *keys, Program *prog, bool isShadowPass1) {
	// Create matrix stacks
	MatrixStack P, V, M;
	// Apply camera transforms
	P.pushMatrix();
	camera->applyProjectionMatrix(&P);
	V.pushMatrix();
	camera->applyViewMatrix(&V);

	// Get light position in Camera space
	Eigen::Vector4f lightPos = V.topMatrix() * Eigen::Vector4f(light.getPosition()(0), light.getPosition()(1), light.getPosition()(2), 1.0f);
	if (!isShadowPass1) {
		glUniform3fv(prog->getUniform("lightPos"), 1, lightPos.data());
		glUniformMatrix4fv(prog->getUniform("P"), 1, GL_FALSE, P.topMatrix().data());
	}

	for (auto &wobj : objects) {
		wobj.draw(M, V, P, prog, light, isShadowPass1);
	}
	track.draw(V, P, prog, light, isShadowPass1);
	vehicle.draw(keys, M, V, P, prog, light, isShadowPass1);

	P.popMatrix();
	V.popMatrix();
}

void Scene::init() {
	for (int i = 0; i < (int)shapes.size(); i++) {
		shapes.at(i).init();
		textures.at(i).init();
	}
	track.init();
	vehicle.init();
	for (auto &wobj : objects) {
		wobj.initPQP();
	}
}

void Scene::update(const bool *keys, const Eigen::Vector2f &mouse, float dt) {
	vehicle.update(keys, mouse, boxes, dt);

	// update the light's target for shadowMapping purposes
	Eigen::Vector3f u(sin(camera->getYaw()), 0.0f, cos(camera->getYaw()));
	light.setTarget(camera->getPosition() + 75.0f * -u);

	checkCollisions();
}

// fills a PQP_REAL matrix with values from a float array matrix
void setPQPRotate(float *raw, PQP_REAL R[3][3]) {
	R[0][0] = raw[0];
	R[0][1] = raw[1];
	R[0][2] = raw[2];
	R[1][0] = raw[3];
	R[1][1] = raw[4];
	R[1][2] = raw[5];
	R[2][0] = raw[6];
	R[2][1] = raw[7];
	R[2][2] = raw[8];
}

// tests 2 objects for collisions using pqp objects
void Scene::pqpCollideWObj(WorldObject &wheel, WorldObject &wobj) {
	PQP_REAL R1[3][3];
	PQP_REAL *T1 = (PQP_REAL *)malloc(3 * sizeof(PQP_REAL));
	PQP_REAL R2[3][3];
	PQP_REAL *T2 = (PQP_REAL *)malloc(3 * sizeof(PQP_REAL));
	
	float pitch = vehicle.getPitch();
	float yaw = vehicle.getYaw();
	Eigen::Vector3f wtraw = wheel.translate;
	std::cout << sin(pitch) * wtraw(1) << std::endl;
	Eigen::Vector3f wtrel(cos(pitch) * sin(yaw) * wtraw(0), sin(pitch) * wtraw(1), cos(pitch) * cos(yaw) * wtraw(2));

	setPQPRotate(wheel.rotate.data(), R1);
	Eigen::Vector3f wp0 = wtrel + vehicle.getPosition();
	wp0(1) -= 0.80f;
	T1 = (PQP_REAL *)(wp0.data());

	setPQPRotate(wobj.rotate.data(), R2);
	T2 = (PQP_REAL *)(wobj.translate.data());

	PQP_CollideResult cres;
	PQP_Collide(&cres, R1, T1, wheel.pqpshape,
					   R2, T2, wobj.pqpshape,
		                  PQP_FIRST_CONTACT);
	if (cres.Colliding()) printf("Collisions: %d\n", cres.NumPairs());
}

// tests collisions between an object and a track
void Scene::pqpCollideTrack(WorldObject &wheel, PQP_Model *track) {
	PQP_REAL R1[3][3];
	PQP_REAL *T1 = (PQP_REAL *)malloc(3 * sizeof(PQP_REAL));
	PQP_REAL R2[3][3] = {{(PQP_REAL)0.0f}};
	R2[0][0] = (PQP_REAL)1.0f;
	R2[1][1] = (PQP_REAL)1.0f;
	R2[2][2] = (PQP_REAL)1.0f;
	PQP_REAL T2[3] = {(PQP_REAL)0.0f};
	
	float pitch = vehicle.getPitch();
	float yaw = vehicle.getYaw();
	Eigen::Vector3f wtraw = wheel.translate;
	Eigen::Vector3f wtrel(cos(pitch) * sin(yaw) * wtraw(0), sin(pitch) * wtraw(1), cos(pitch) * cos(yaw) * wtraw(2));

	setPQPRotate(wheel.rotate.data(), R1);
	Eigen::Vector3f wp0 = wtrel + vehicle.getPosition();
	wp0(1) -= 0.90f;
	T1 = (PQP_REAL *)(wp0.data());

	PQP_CollideResult cres;
	PQP_Collide(&cres, R1, T1, wheel.pqpshape,
					   R2, T2, track,
		                  PQP_FIRST_CONTACT);
	if (cres.Colliding()) printf("Collisions: %d\n", cres.NumPairs());
}

void Scene::checkCollisions() {
	for (auto &w : vehicle.wheels) {
		for (auto &wobj : objects) {
			pqpCollideWObj(w, wobj);
		}
		pqpCollideTrack(w, track.pqpshape);
	}
}

// Loads the objects with transformations and materials
void Scene::load(const char *filename) {
	ifstream in;
	in.open(filename);
	if(!in.good()) {
		std::cout << "Cannot read " << filename << endl;
		exit(1);
	}
	
	int numShapes;
	in >> numShapes;
	shapes.resize(numShapes);
	textures.resize(numShapes);
	shapespertex.resize(numShapes);

	string line;
	getline(in, line);

	int currentShape = 0;
	while(1) {
		getline(in, line);
		if(in.eof()) {
			break;
		}
		// Skip empty lines
		if(line.size() < 2) {
			continue;
		}
		// Skip comments
		if(line.at(0) == '#') {
			continue;
		}
		// Parse line
		stringstream ss(line);
		string mesh;
		string texfile;
		int numObjs;
		ss >> mesh;
		ss >> texfile;
		ss >> numObjs;
		Shape tempShape;
		tempShape.load(mesh);
		Texture tex;
		tex.setFilename(texfile);
		textures.at(currentShape) = tex;
		shapes.at(currentShape) = tempShape;
		//shapespertex.at(currentShape) = numObjs;

		// get transformations of each object
		for (int i = 0; i < numObjs; i++) {
			WorldObject newObj;

			// getting and setting translation
			getline(in, line);
			stringstream ss0(line);
			float tx, ty, tz;
			ss0 >> tx;
			ss0 >> ty;
			ss0 >> tz;
			newObj.setTranslate(Eigen::Vector3f(tx, ty, tz));

			// getting and setting scale
			getline(in, line);
			stringstream ss1(line);
			float sx, sy, sz;
			ss1 >> sx;
			ss1 >> sy;
			ss1 >> sz;
			newObj.setScale(Eigen::Vector3f(sx, sy, sz));

			// getting and setting rotation
			getline(in, line);
			stringstream ss2(line);
			float rmag, rx, ry, rz;
			ss2 >> rmag;
			ss2 >> rx;
			ss2 >> ry;
			ss2 >> rz;
			newObj.setRotate(Eigen::AngleAxisf(rmag * M_PI, Eigen::Vector3f(rx, ry, rz)).toRotationMatrix());
			newObj.setShape(&shapes.at(currentShape));
			newObj.setTexture(&textures.at(currentShape));

			getline(in, line);
			stringstream ss3(line);
			int isCollidable;
			char x, y;
			ss3 >> isCollidable;
			ss3 >> x;
			ss3 >> y;

			// texture scaling info for proper texture stretching
			float texX, texY;
			switch(x) {
				case 'x': texX = sx;
					break;
				case 'y': texX = sy;
					break;
				case 'z': texX = sz;
					break;
			}
			switch(y) {
				case 'x': texY = sx;
					break;
				case 'y': texY = sy;
					break;
				case 'z': texY = sz;
					break;
			}
			newObj.buildTexMatrix(texX, texY);
			objects.push_back(newObj);

			// generate a collision box if the object is set to collidable
			if (isCollidable == 1) {
				CollisionBox tempBox;
				tempBox.setBounds(tx - 0.5f * sx, tx + 0.5f * sx,
										 ty - 0.5f * sy, ty + 0.5f * sy,
										 tz - 0.5f * sz, tz + 0.5f * sz);
				boxes.push_back(tempBox);
			}
		}
		currentShape++;
	}
	in.close();
	vehicle.load();
	track.load("../materials/mytrack.txt");
}
