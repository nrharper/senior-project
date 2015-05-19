#pragma  once
#ifndef __WorldObject__
#define __WorldObject__

#ifdef __APPLE__
#include <GLUT/glut.h>
#endif
#ifdef __unix__
#include <GL/glut.h>
#endif
#include <stdio.h>
#include <cmath>
#include "GLSL.h"
#include "MatrixStack.h"
#include "Shape.h"
#include "Program.h"
#include "../libraries/Light.h"
#include "PQP.h"
#include "../libraries/Texture.h"

class WorldObject {
public:
	WorldObject();
	virtual ~WorldObject();

	Eigen::Vector3f translate;
	Eigen::Matrix3f rotate;
	PQP_Model *pqpshape;

	void init(void);
	void draw(MatrixStack &M, MatrixStack &V, MatrixStack &P, Program *prog, Light &light, bool isShadowPass1) const;
	void buildTexMatrix(float, float);
	Eigen::Matrix3f &getTexMatrix() { return texMat; }
	void setShape(Shape *s) { shape = s; }
	void setTexture(Texture *t) { texture = t; }
	Shape* getShape() { return shape; }
	void setScale(Eigen::Vector3f s) { scale = s; }
	void setTranslate(Eigen::Vector3f t) { translate = t; }
	void setRotate(Eigen::Matrix3f r) { rotate = r; }
	void initPQP();

private:
	Eigen::Matrix3f texMat;
	Eigen::Vector3f diffuse;
	Shape *shape;
	Eigen::Vector3f scale;
	Texture *texture;
};

#endif
