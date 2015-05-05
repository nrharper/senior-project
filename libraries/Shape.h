#pragma once
#ifndef _SHAPE_H_
#define _SHAPE_H_

#ifdef __APPLE__
#include <GLUT/glut.h>
#endif
#ifdef __unix__
#include <GL/glut.h>
#endif
#include "tiny_obj_loader.h"

class Shape
{
public:
	Shape();
	virtual ~Shape();
	void load(const std::string &meshName);
	void init();
	void draw(GLint h_pos, GLint h_nor, GLint h_tex);
	
private:
	std::vector<tinyobj::shape_t> shapes;
	GLuint posBufID;
	GLuint norBufID;
	GLuint texBufID;
	GLuint indBufID;
};

#endif
