//
// sueda
// November, 2014
//

#pragma  once
#ifndef __Camera__
#define __Camera__

#define GLM_FORCE_RADIANS
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "MatrixStack.h"

class Camera {
public:
	
	Camera();
	virtual ~Camera();
	void setWindowSize(float w, float h);
	void update(float yaw, float pitch, Eigen::Vector3f pos);
	void applyProjectionMatrix(MatrixStack *P) const;
	void applyViewMatrix(MatrixStack *MV) const;
	float getYaw() const { return yawRotation; }
	float getPitch() const { return pitchRotation; }
	float getVelocity() const { return velocity; }
	Eigen::Vector3f getPosition() const { return position; }
	
private:
	float aspect;
	float fovy;
	float znear;
	float zfar;
	float yawRotation;
	float pitchRotation;
	float velocity;
	Eigen::Vector2f mousePrev;
	Eigen::Vector3f position;
};

#endif
