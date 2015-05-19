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
	float &getVelocity() { return speed; }
	Eigen::Vector3f &getPosition() { return position; }
	
private:
	float mass;
	float momentx;
	float momenty;
	float yawVelocity;
	float pitchVelocity;
	Eigen::Vector3f torque;
	Eigen::Vector3f force;

	// driving variables
	float yaw;
	float pitch;
	float tireSpin;
	float speed;
	float steerAngle;
	float wheelBase;
	Eigen::Vector2f mousePrev;
	Eigen::Vector3f position;
	Eigen::Vector3f velocity;

	Shape chasis_shape;
	Shape wheel_shape;
	Texture ctex;
	Texture wtex;
};

#endif
