#include "../libraries/Vehicle.h"
#include <iostream>

using namespace std;

Vehicle::Vehicle() :
	mass(1500.0f),
	yawVelocity(0.0f),
	pitchVelocity(0.0f),
	yaw((float)M_PI),
   pitch(0.0f),
	tireSpin(0.0f),
	vehicleAngle((float)M_PI),
   speed(0.0f),
	steerAngle(0.0f),
	wheelBase(2.0f),
   mousePrev(200.0f, 200.0f),
	position(00.0f, 0.9f, -180.0f),
	velocity(0.0f, 0.0f, 0.0f)
{
}

Vehicle::~Vehicle()
{
	
}

void Vehicle::load() {
	chasis_shape.load("../materials/cube.obj");
	wheel_shape.load("../materials/tire.obj");

	wheel.setShape(&wheel_shape);
	Eigen::Matrix4f R = Eigen::Matrix4f::Identity();
	R.block<3,3>(0,0) = Eigen::AngleAxisf((float)M_PI / 2, Eigen::Vector3f(0.0f, 1.0f, 0.0f)).toRotationMatrix();
	Eigen::Matrix4f S = Eigen::Matrix4f::Identity();
	S(0,0) = 0.75f;
	S(1,1) = 0.75f;
	S(2,2) = 0.25f;
	wheel.setTransform(R * S);

	chasis.setShape(&chasis_shape);
	S = Eigen::Matrix4f::Identity();
	S(1,1) = 0.4f;
	chasis.setTransform(S);

	computeMomentsOfInertia();
}

void Vehicle::init() {
	chasis_shape.init();
	wheel_shape.init();
}

bool checkCollisions(Eigen::Vector3f pos, float yaw, const std::vector<CollisionBox> &boxes) {
	// Make vehicle collision boxes
   CollisionBox temp1, temp2;
   Eigen::Vector3f tempPos = Eigen::Vector3f(pos(0) + sin(yaw), pos(1), pos(2) + cos(yaw));
   temp1.setBounds(tempPos(0) - 0.5, tempPos(0) + 0.5, tempPos(1) + 0.5, tempPos(1) + 1.5, tempPos(2) - 0.5, tempPos(2) + 0.5);
   tempPos = Eigen::Vector3f(pos(0) - sin(yaw), pos(1), pos(2) - cos(yaw));
   temp2.setBounds(tempPos(0) - 0.5, tempPos(0) + 0.5, tempPos(1) + 0.5, tempPos(1) + 1.5, tempPos(2) - 0.5, tempPos(2) + 0.5);

	// Check for collisions
   bool isCollision = false;
   for (int i = 0; i < (int)boxes.size(); i++) {
      if (temp1.isXZCollision(boxes.at(i)) || temp2.isXZCollision(boxes.at(i))) {
         isCollision = true;
      }
   }

	return isCollision;
}

void Vehicle::computeMomentsOfInertia() {
	momentx = mass * (0.8f * 0.8f + 2.0f * 2.0f) / 12.0f;
	momenty = mass * (2.0f * 2.0f + 2.0f * 2.0f) / 12.0f;
}

void Vehicle::computeForces(const bool *keys) {
	Eigen::Vector3f a, f, Fd, Fr, u;
	Eigen::Vector3f Fa(0.0f, 0.0f, 0.0f);
	u(0) = sin(steerAngle);
	u(1) = 0.0f;
	u(2) = cos(steerAngle);
	u.normalize();
	float engineForce = 60000.0f;
	if (keys['w']) {
		Fa = u * engineForce;
		Fr = 0.0f * velocity;
	}
	else {
		if (keys['s'])
			Fa = -u * 10000.0f;
		Fr = -125.0f * velocity;
	}
	Fd = -50.00f * velocity * velocity.norm();
	f = Fa + Fd + Fr;
	
	force = 2.0f * f;

	torque = Eigen::Vector3f(0.5f, 0.0f, -0.5f).cross(f) + Eigen::Vector3f(-0.5f, 0.0f, -0.5f).cross(f);
}

void Vehicle::update(const bool *keys, const Eigen::Vector2f &mouse, const std::vector<CollisionBox> &boxes, float dt) {
	// Turning
   if (keys['a']) {
		if (steerAngle < (float)M_PI / 4)
	      steerAngle += ((float)M_PI / 100);
   }
   else if (keys['d']) {
		if (steerAngle > (float)-M_PI / 4)
	      steerAngle -= ((float)M_PI / 100);
   }
	else
		steerAngle = 0.5f * steerAngle; 

	computeForces(keys);

	Eigen::Vector3f acceleration = force / mass;
	velocity += acceleration * dt;
	speed = velocity.norm();
	Eigen::Vector3f pos = position;
	Eigen::Vector3f velocityDir(velocity(0) * sin(steerAngle), velocity(1), velocity(2) * cos(steerAngle));
	pos -= speed * dt * Eigen::Vector3f(sin(yaw), 0, cos(yaw));

	float yawAcceleration = torque(1) / momenty / 6;
	float pitchAcceleration = torque(0) / momentx / 6;
	yawVelocity -= yawAcceleration * dt;
	pitchVelocity -= pitchAcceleration * dt;
	yaw += yawVelocity * dt;
	pitch += pitchVelocity * dt;

	// Update to new orientation if no collision
   if (!checkCollisions(pos, vehicleAngle, boxes)) {
      position = Eigen::Vector3f(pos(0), pos(1), pos(2));
		//yaw = vehicleAngle;
   }
   else {
      velocity << 0.0f, 0.0f, 0.0f;
      yawVelocity = 0.0f;
	}
}

void Vehicle::draw(const bool *keys, MatrixStack &MV, Program *prog) {
	if (abs(speed) > 0)
      tireSpin -= speed;

	// Transform and draw
	MV.pushMatrix();
      MV.translate(position);
      MV.translate(Eigen::Vector3f(0.0f, -1.0f, 0.0f));
      MV.rotate(yaw, Eigen::Vector3f(0.0f, 1.0f, 0.0f));
      MV.pushMatrix(); // front left tire
         MV.translate(Eigen::Vector3f(-0.5f, 0.25f, -0.8f));
         MV.rotate(steerAngle, Eigen::Vector3f(0.0f, 1.0f, 0.0f));
         MV.rotate(tireSpin, Eigen::Vector3f(1.0f, 0.0f, 0.0f));
         wheel.draw(MV, prog);
      MV.popMatrix();
      MV.pushMatrix(); // front right tire
         MV.translate(Eigen::Vector3f(0.5f, 0.25f, -0.8f));
         MV.rotate(steerAngle, Eigen::Vector3f(0.0f, 1.0f, 0.0f));
         MV.rotate(tireSpin, Eigen::Vector3f(1.0f, 0.0f, 0.0f));
         wheel.draw(MV, prog);
      MV.popMatrix();
      MV.pushMatrix(); // rear left tire
         MV.translate(Eigen::Vector3f(-0.5f, 0.25f, 0.8f));
         MV.rotate(tireSpin, Eigen::Vector3f(1.0f, 0.0f, 0.0f));
         wheel.draw(MV, prog);
      MV.popMatrix();
      MV.pushMatrix(); // rear right tire
         MV.translate(Eigen::Vector3f(0.5f, 0.25f, 0.8f));
         MV.rotate(tireSpin, Eigen::Vector3f(1.0f, 0.0f, 0.0f));
         wheel.draw(MV, prog);
      MV.popMatrix();
      MV.pushMatrix(); // chasis
         MV.translate(Eigen::Vector3f(0.0f, 0.25f, 0.0f));
	      chasis.draw(MV, prog);
      MV.popMatrix();
   MV.popMatrix();
}
