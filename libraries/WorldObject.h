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

class WorldObject {
public:
	WorldObject();
	virtual ~WorldObject();

	void init(void);
	void draw(MatrixStack &MV, Program *prog) const;
	void buildTexMatrix(float, float);
	Eigen::Matrix3f &getTexMatrix() { return texMat; }
	void setShape(Shape *s) { shape = s; }
	Shape* getShape() { return shape; }
	void setTransform(Eigen::Matrix4f t) { transform = t; }

private:
	Eigen::Matrix3f texMat;
	Eigen::Vector3f diffuse;
	Shape *shape;
	Eigen::Matrix4f transform;
};

#endif
