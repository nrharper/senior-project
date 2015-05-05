#include "../libraries/KartRacer.h"

float randFloat(float l, float h)
{
	float r = rand() / (float)RAND_MAX;
	return (1.0f - r) * l + r * h;
}

WorldObject::WorldObject() :
   diffuse(randFloat(0.2f, 0.8f), randFloat(0.2f, 0.8f), randFloat(0.2f, 0.8f))
{
}


WorldObject::~WorldObject()
{
	
}

void WorldObject::draw(MatrixStack &MV, Program *prog) const {
   MV.pushMatrix();
	MV.multMatrix(transform);

	glUniform3fv(prog->getUniform("kd"),  1, diffuse.data());
	glUniformMatrix4fv(prog->getUniform("MV"), 1, GL_FALSE, MV.topMatrix().data());
	glUniformMatrix3fv(prog->getUniform("T1"), 1, GL_TRUE, texMat.data());
	shape->draw(prog->getAttribute("vertPos"), prog->getAttribute("vertNor"), prog->getAttribute("vertTex"));

   MV.popMatrix();
}

void WorldObject::buildTexMatrix(float scaleX, float scaleY) {
	texMat = Eigen::Matrix3f::Identity();
	texMat(0,0) = scaleX / 5;
	texMat(1,1) = scaleY / 5;
}
