#pragma once
#ifndef _SHAPEOBJ_H_
#define _SHAPEOBJ_H_

#include "tiny_obj_loader.h"

class ShapeObj
{
public:
	ShapeObj();
	virtual ~ShapeObj();
	void load(const std::string &meshName);
	void init();
	void draw(int h_pos, int h_nor, int h_tex) const;
	
private:
	std::vector<tinyobj::shape_t> shapes;
	unsigned posBufID;
	unsigned norBufID;
	unsigned texBufID;
	unsigned indBufID;
};

#endif
