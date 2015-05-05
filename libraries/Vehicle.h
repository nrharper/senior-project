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

class Vehicle {
public:
	Vehicle();
	virtual ~Vehicle();
	void load();
	void init();
	void computeMomentsOfInertia();
	void computeForces(const bool *keys);
	void update(const bool *keys, const Eigen::Vector2f &mouse, const std::vector<CollisionBox> &boxes, float dt);
	void draw(const bool *keys, MatrixStack &MV, Program *prog);
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
	Eigen::Vector3f force;
	Eigen::Vector3f torque;

	// driving variables
	float yaw;
	float pitch;
	float tireSpin;
	float vehicleAngle;
	float speed;
	float steerAngle;
	float wheelBase;
	Eigen::Vector2f mousePrev;
	Eigen::Vector3f position;
	Eigen::Vector3f velocity;

	WorldObject wheel;
	WorldObject chasis;
	Shape chasis_shape;
	Shape wheel_shape;
};

#endif
