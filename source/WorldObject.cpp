#include <cstdlib>
#include <iostream>
#include "../libraries/WorldObject.h"

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

void WorldObject::initPQP() {
	pqpshape = new PQP_Model();
	PQP_REAL p1[3],p2[3],p3[3];

	std::vector<float> positions = shape->getShape().mesh.positions;
	std::vector<unsigned int> indices = shape->getShape().mesh.indices;
	int ntris = (int)indices.size();
	pqpshape->BeginModel();
	for (int i = 0; i < ntris; i += 3) {
		p1[0] = (PQP_REAL)(positions[indices[i] * 3 + 0] * scale(0));
		p1[1] = (PQP_REAL)(positions[indices[i] * 3 + 1] * scale(1));
		p1[2] = (PQP_REAL)(positions[indices[i] * 3 + 2] * scale(2));
		p2[0] = (PQP_REAL)(positions[indices[i + 1] * 3 + 0] * scale(0));
		p2[1] = (PQP_REAL)(positions[indices[i + 1] * 3 + 1] * scale(1));
		p2[2] = (PQP_REAL)(positions[indices[i + 1] * 3 + 2] * scale(2));
		p3[0] = (PQP_REAL)(positions[indices[i + 2] * 3 + 0] * scale(0));
		p3[1] = (PQP_REAL)(positions[indices[i + 2] * 3 + 1] * scale(1));
		p3[2] = (PQP_REAL)(positions[indices[i + 2] * 3 + 2] * scale(2));
		pqpshape->AddTri(p1,p2,p3, i / 3);
	}
	pqpshape->EndModel();
}

void WorldObject::draw(MatrixStack &MV, MatrixStack &P, Program *prog, Light &light, bool isShadowPass1) const {
	//float extra = (float)M_PI/8.0f;

	MatrixStack lightP, lightMV; // light matrices
	lightP.pushMatrix();
	light.applyProjectionMatrix(&lightP);
	lightMV.pushMatrix();
	light.applyViewMatrix(&lightMV);
	lightMV.translate(translate);
	//lightMV.rotate(yaw + extra, Eigen::Vector3f(0.0f, 1.0f, 0.0f));
	lightMV.scale(scale);

	Eigen::Matrix4f lightMVP = lightP.topMatrix() * lightMV.topMatrix();

	if (isShadowPass1) {
		glUniformMatrix4fv(prog->getUniform("MVP"), 1, GL_FALSE, lightMVP.data());
		shape->draw(prog->getAttribute("vertPos"), -1, -1);
		return;
	}

	MV.pushMatrix();
	MV.translate(translate);
	//MV.rotate(yaw + extra, Eigen::Vector3f(0.0f, 1.0f, 0.0f));
	MV.scale(scale);
	glUniformMatrix4fv(prog->getUniform("lightMVP"), 1, GL_FALSE, lightMVP.data());
	glUniform3fv(prog->getUniform("kd"),  1, diffuse.data());
	glUniformMatrix4fv(prog->getUniform("MV"), 1, GL_FALSE, MV.topMatrix().data());
	glUniformMatrix3fv(prog->getUniform("Tscale"), 1, GL_TRUE, texMat.data());

	texture->bind(prog->getUniform("texture"), 1);	
	shape->draw(prog->getAttribute("vertPos"), prog->getAttribute("vertNor"), prog->getAttribute("vertTex"));
	texture->unbind(1);

	MV.popMatrix();
	lightMV.popMatrix();
	lightP.popMatrix();
}

void WorldObject::buildTexMatrix(float scaleX, float scaleY) {
	texMat = Eigen::Matrix3f::Identity();
	texMat(0,0) = scaleX / 5;
	texMat(1,1) = scaleY / 5;
}
