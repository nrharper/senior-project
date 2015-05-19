#include "../libraries/Light.h"
#include "../libraries/MatrixStack.h"
#include <iostream>

Light::Light() :
	aspect(1.0f),
	fovy(20.0f/180.0f*M_PI),
	znear(1.0f),
	zfar(1300.0f),
	position(500.0f, 500.0f, 500.0f),
	target(0.0f, 0.0f, 0.0f),
	up(0.0f, 1.0f, 0.0f)
{
}

Light::~Light()
{
	
}

void Light::applyProjectionMatrix(MatrixStack *P) const
{
	P->perspective(fovy, aspect, znear, zfar);
}

void Light::applyViewMatrix(MatrixStack *MV) const
{
	MV->lookAt(position, target, up);
}
