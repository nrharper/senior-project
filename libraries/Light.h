#pragma  once
#ifndef __Light__
#define __Light__

#include <Eigen/Dense>
#include <Eigen/Geometry>

class MatrixStack;

class Light
{
public:
	Light();
	virtual ~Light();
	float getAspect() const { return aspect; }
	float getFovy() const { return fovy; }
	float getZnear() const { return znear; }
	float getZfar() const { return zfar; }
	void setAspect(float a) { aspect = a; }
	void setFovy(float f) { fovy = f; }
	void setZnear(float n) { znear = n; }
	void setZfar(float f) { zfar = f; }
	const Eigen::Vector3f &getPosition() const { return position; }
	const Eigen::Vector3f &getTarget() const { return target; }
	const Eigen::Vector3f &getUp() const { return up; }
	void setPosition(const Eigen::Vector3f &p) { position = p; }
	void setTarget(const Eigen::Vector3f &t) { target = t; }
	void setUp(const Eigen::Vector3f &u) { up = u; }
	void applyProjectionMatrix(MatrixStack *P) const;
	void applyViewMatrix(MatrixStack *MV) const;
	
private:
	float aspect;
	float fovy;
	float znear;
	float zfar;
	Eigen::Vector3f position;
	Eigen::Vector3f target;
	Eigen::Vector3f up;
};

#endif
