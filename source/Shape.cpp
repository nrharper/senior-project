#include <iostream>
#define GLM_FORCE_RADIANS
#include <Eigen/Dense>
#include "../libraries/Shape.h"
#include "../libraries/GLSL.h"

using namespace std;

Shape::Shape() :
	posBufID(0),
	norBufID(0),
	texBufID(0),
	indBufID(0)
{
}

Shape::~Shape()
{
}

void Shape::load(const string &meshName)
{
	// Load geometry
	// Some obj files contain material information.
	// We'll ignore them for this assignment.
	std::vector<tinyobj::material_t> objMaterials;
	string err = tinyobj::LoadObj(shapes, objMaterials, meshName.c_str());
	if(!err.empty()) {
		cerr << err << endl;
	}
	
	// Scale the vertex positions so that they fit within [-1, +1] in all three dimensions.
	vector<float> &posBuf = shapes[0].mesh.positions;
	Eigen::Vector3f vmin(posBuf[0], posBuf[1], posBuf[2]);
	Eigen::Vector3f vmax(posBuf[0], posBuf[1], posBuf[2]);
	for(int i = 3; i < (int)posBuf.size(); i += 3) {
		Eigen::Vector3f v(posBuf[i], posBuf[i+1], posBuf[i+2]);
		vmin(0) = min(vmin(0), v(0));
		vmin(1) = min(vmin(1), v(1));
		vmin(2) = min(vmin(2), v(2));
		vmax(0) = max(vmax(0), v(0));
		vmax(1) = max(vmax(1), v(1));
		vmax(2) = max(vmax(2), v(2));
	}
	Eigen::Vector3f center = 0.5f*(vmin + vmax);
	Eigen::Vector3f diff = vmax - vmin;
	float diffmax = diff(0);
	diffmax = max(diffmax, diff(1));
	diffmax = max(diffmax, diff(2));
	float scale = 1.0f / diffmax;
	for(int i = 0; i < (int)posBuf.size(); i += 3) {
		posBuf[i  ] = (posBuf[i  ] - center(0)) * scale;
		posBuf[i+1] = (posBuf[i+1] - center(1)) * scale;
		posBuf[i+2] = (posBuf[i+2] - center(2)) * scale;
	}
}

void Shape::init()
{
	// Send the position array to the GPU
	const vector<float> &posBuf = shapes[0].mesh.positions;
	glGenBuffers(1, &posBufID);
	glBindBuffer(GL_ARRAY_BUFFER, posBufID);
	glBufferData(GL_ARRAY_BUFFER, posBuf.size()*sizeof(float), &posBuf[0], GL_STATIC_DRAW);
	
	// Send the normal array (if it exists) to the GPU
	const vector<float> &norBuf = shapes[0].mesh.normals;
	if(!norBuf.empty()) {
		glGenBuffers(1, &norBufID);
		glBindBuffer(GL_ARRAY_BUFFER, norBufID);
		glBufferData(GL_ARRAY_BUFFER, norBuf.size()*sizeof(float), &norBuf[0], GL_STATIC_DRAW);
	} else {
		norBufID = 0;
	}
	
	// Send the texture coordinates array (if it exists) to the GPU
	const vector<float> &texBuf = shapes[0].mesh.texcoords;
	if(!texBuf.empty()) {
		glGenBuffers(1, &texBufID);
		glBindBuffer(GL_ARRAY_BUFFER, texBufID);
		glBufferData(GL_ARRAY_BUFFER, texBuf.size()*sizeof(float), &texBuf[0], GL_STATIC_DRAW);
	} else {
		texBufID = 0;
	}
	
	// Send the index array to the GPU
	const vector<unsigned int> &indBuf = shapes[0].mesh.indices;
	glGenBuffers(1, &indBufID);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indBufID);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indBuf.size()*sizeof(unsigned int), &indBuf[0], GL_STATIC_DRAW);
	
	// Unbind the arrays
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	
	assert(glGetError() == GL_NO_ERROR);
}

void Shape::draw(GLint h_pos, GLint h_nor, GLint h_tex)
{
	// Enable and bind position array for drawing
	GLSL::enableVertexAttribArray(h_pos);
	glBindBuffer(GL_ARRAY_BUFFER, posBufID);
	glVertexAttribPointer(h_pos, 3, GL_FLOAT, GL_FALSE, 0, 0);
	
	// Enable and bind normal array (if it exists) for drawing
	if(norBufID && h_nor >= 0) {
		GLSL::enableVertexAttribArray(h_nor);
		glBindBuffer(GL_ARRAY_BUFFER, norBufID);
		glVertexAttribPointer(h_nor, 3, GL_FLOAT, GL_FALSE, 0, 0);
	}
	
	// Enable and bind texcoord array (if it exists) for drawing
	if(texBufID && h_tex >= 0) {
		GLSL::enableVertexAttribArray(h_tex);
		glBindBuffer(GL_ARRAY_BUFFER, texBufID);
		glVertexAttribPointer(h_tex, 2, GL_FLOAT, GL_FALSE, 0, 0);
	}
	
	// Bind index array for drawing
	int nIndices = (int)shapes[0].mesh.indices.size();
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indBufID);
	
	// Draw
	glDrawElements(GL_TRIANGLES, nIndices, GL_UNSIGNED_INT, 0);
	
	// Disable and unbind
	if(texBufID && h_tex >= 0) {
		GLSL::disableVertexAttribArray(h_tex);
	}
	if(norBufID && h_nor >= 0) {
		GLSL::disableVertexAttribArray(h_nor);
	}
	GLSL::disableVertexAttribArray(h_pos);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}
