#include "../libraries/Scene.h"
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

Scene::~Scene()
{
	
}

void Scene::draw(const bool *keys, MatrixStack &MV, Program *prog) {
	int texnum = 0, lastnum = 0;
	for (auto &num : shapespertex) {
		textures.at(texnum).bind(prog->getUniform("texture"), texnum);
		for (int i = lastnum; i < lastnum + num; i++) {
			objects.at(i).draw(MV, prog);
		}
		lastnum += num;
		textures.at(texnum).unbind(texnum);
		texnum++;
	}
	track.draw(MV, prog);
	vehicle.draw(keys, MV, prog);
}

void Scene::init() {
	for (int i = 0; i < (int)shapes.size(); i++) {
		shapes.at(i).init();
		textures.at(i).init();
	}
	track.init();
	vehicle.init();
}

void Scene::update(const bool *keys, const Eigen::Vector2f &mouse, float dt) {
	vehicle.update(keys, mouse, boxes, dt);
}

// Loads the attachment file (bone weights for each vertex)
void Scene::load(const char *filename) {
	ifstream in;
	in.open(filename);
	if(!in.good()) {
		std::cout << "Cannot read " << filename << endl;
		exit(1);
	}

	string line;
	// Discard the first 2 lines
	//getline(in, line);
	//getline(in, line);
	
	int numShapes;
	in >> numShapes;
	shapes.resize(numShapes);
	textures.resize(numShapes);
	shapespertex.resize(numShapes);
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
		shapespertex.at(currentShape) = numObjs;

		// get every bone weight for current vertex
		for (int i = 0; i < numObjs; i++) {
			WorldObject newObj;

			getline(in, line);
			stringstream ss0(line);
			float tx, ty, tz; // First get translation
			ss0 >> tx;
			ss0 >> ty;
			ss0 >> tz;
			Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
			T(0,3) = tx;
			T(1,3) = ty;
			T(2,3) = tz;

			getline(in, line);
			stringstream ss1(line);
			float sx, sy, sz; // Next get scale
			ss1 >> sx;
			ss1 >> sy;
			ss1 >> sz;
			Eigen::Matrix4f S = Eigen::Matrix4f::Identity();
			S(0,0) = sx;
			S(1,1) = sy;
			S(2,2) = sz;

			getline(in, line);
			stringstream ss2(line);
			float rmag, rx, ry, rz;
			ss2 >> rmag;
			ss2 >> rx;
			ss2 >> ry;
			ss2 >> rz;
			Eigen::Matrix4f R = Eigen::Matrix4f::Identity();
			R.block<3,3>(0,0) = Eigen::AngleAxisf(rmag * M_PI, Eigen::Vector3f(rx, ry, rz)).toRotationMatrix();

			newObj.setTransform(T * S);
			newObj.setShape(&shapes.at(currentShape));

			getline(in, line);
			stringstream ss3(line);
			int isCollidable;
			char x, y;
			ss3 >> isCollidable;
			ss3 >> x;
			ss3 >> y;
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
