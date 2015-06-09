#pragma  once
#ifndef __Vehicle__
#define __Vehicle__

#define GLM_FORCE_RADIANS
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "WorldObject.h"
#include "Program.h"
#include "MatrixStack.h"
#include "CollisionBox.h"
#include "Shape.h"
#include "Light.h"

class Vehicle {
public:

	std::vector<WorldObject> wheels;
	std::vector<Eigen::Vector3f> wforces;
	WorldObject chasis;

	float momentx;
	float momenty;
	float momentz;

	float yawVelocity;
	float pitchVelocity;
	float rollVelocity;

	Vehicle();
	virtual ~Vehicle();
	void load();
	void init();
	void computeMomentsOfInertia();
	void computeForces(const bool *keys, float dt);
	void update(const bool *keys, const Eigen::Vector2f &mouse, const std::vector<CollisionBox> &boxes, float dt);
	void draw(const bool *keys, MatrixStack &M, MatrixStack &V, MatrixStack &P, Program *prog, Light &light, bool isShadowPass1);
	float &getYaw() { return yaw; }
	float &getPitch() { return pitch; }
	float &getRoll() { return roll; }
	Eigen::Vector3f &getVelocity() { return velocity; }
	Eigen::Vector3f &getPosition() { return position; }
	float &getMass() { return mass; }
	void setVelocity(Eigen::Vector3f v) { velocity = v; }
	
private:
	float mass;
	Eigen::Vector3f torque;
	Eigen::Vector3f force;
	Eigen::Vector3f forceT;

	// driving variables
	float yaw;
	float pitch;
	float roll;
	float tireSpin;
	float steerAngle;
	Eigen::Vector2f mousePrev;
	Eigen::Vector3f position;
	Eigen::Vector3f velocity;
	Eigen::Vector3f velocityT;

	Shape chasis_shape;
	Shape wheel_shape;
	Texture ctex;
	Texture wtex;
};

#endif
