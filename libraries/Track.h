#pragma once

#ifndef __Track__
#define __Track__

#include <vector>
#include <Eigen/Dense>
#include "Program.h"
#include "MatrixStack.h"
#include "GLSL.h"
#include "Texture.h"

class Track {
public:
	virtual ~Track();
	Track();

	struct MeshSegment {
		unsigned posBufID;
		unsigned norBufID;
		unsigned texBufID;
		unsigned indBufID;
	 	std::vector<float>          positions;
		std::vector<float>          normals;
		std::vector<float>          texcoords;
		std::vector<unsigned int>   indices;
	} ;

	void init();
	void initMesh(MeshSegment &mesh);
	void buildGeometry();
	void buildTable();
	float s2u(float s);
	void load(const char *filename);
	void draw(MatrixStack &MV, Program *prog);
	void drawMesh(Program *prog, MeshSegment &mesh);
	
private:
	
	float curveLength;
	Eigen::Matrix4f B;

	// Control points
	struct KeyFrame {
		Eigen::Vector3f cp;
		Eigen::Vector3f n;
	} ;

	Texture toptex;
	Texture bottomtex;

	std::vector<KeyFrame> cps;

	std::vector<std::pair<float,float> > usTable;

	std::vector<MeshSegment> topMeshes;
	std::vector<MeshSegment> bottomMeshes;
	MeshSegment collisionMesh;
};

#endif
