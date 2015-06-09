#include "../libraries/Vehicle.h"
#include "../libraries/Shape.h"
#include <iostream>

using namespace std;

Vehicle::Vehicle() :
	yawVelocity(0.0f),
	pitchVelocity(0.0f),
	mass(1500.0f),
	yaw((float)M_PI),
	pitch(0.0f),
	tireSpin(0.0f),
	steerAngle(0.0f),
	mousePrev(200.0f, 200.0f),
	position(0.0f, 14.0f, -180.0f),
	velocity(0.0f, 0.0f, 0.0f)
{
}

Vehicle::~Vehicle()
{
	
}


// set up the vehicle
void Vehicle::load() {
	chasis_shape.load("../materials/UFO.obj");
	wheel_shape.load("../materials/sphere.obj");
	ctex.setFilename("../materials/concrete_smooth.jpg");
	wtex.setFilename("../materials/dirt.jpg");

	Eigen::Matrix3f identity = Eigen::Matrix3f::Identity();

	WorldObject wheel0;
	wheel0.setShape(&wheel_shape);
	wheel0.setTexture(&wtex);
	wheel0.setTranslate(Eigen::Vector3f(-0.5f, 0.25f, -0.8f));
	wheel0.setScale(Eigen::Vector3f(0.75f, 0.75f, 0.75f));
	wheel0.setRotate(identity);
	wheel0.buildTexMatrix(0.75f, 0.75f);
	wheels.push_back(wheel0);

	WorldObject wheel1;
	wheel1.setShape(&wheel_shape);
	wheel1.setTexture(&wtex);
	wheel1.setTranslate(Eigen::Vector3f(0.5f, 0.25f, -0.8f));
	wheel1.setScale(Eigen::Vector3f(0.75f, 0.75f, 0.75f));
	wheel1.setRotate(identity);
	wheel1.buildTexMatrix(0.75f, 0.75f);
	wheels.push_back(wheel1);

	WorldObject wheel2;
	wheel2.setShape(&wheel_shape);
	wheel2.setTexture(&wtex);
	wheel2.setTranslate(Eigen::Vector3f(-0.5f, 0.25f, 0.8f));
	wheel2.setScale(Eigen::Vector3f(0.75f, 0.75f, 0.75f));
	wheel2.setRotate(identity);
	wheel2.buildTexMatrix(0.75f, 0.75f);
	wheels.push_back(wheel2);

	WorldObject wheel3;
	wheel3.setShape(&wheel_shape);
	wheel3.setTexture(&wtex);
	wheel3.setTranslate(Eigen::Vector3f(0.5f, 0.25f, 0.8f));
	wheel3.setScale(Eigen::Vector3f(0.75f, 0.75f, 0.75f));
	wheel3.setRotate(identity);
	wheel3.buildTexMatrix(0.75f, 0.75f);
	wheels.push_back(wheel3);

	Eigen::Vector3f wforce(0.0f, 0.0f, 0.0f);
	for (unsigned int i = 0; i < 4; i++) {
		wforces.push_back(wforce);
	}

	chasis.setShape(&chasis_shape);
	chasis.setTexture(&ctex);
	chasis.setTranslate(Eigen::Vector3f(0.0f, 0.25f, 0.0f));
	chasis.setScale(Eigen::Vector3f(2.0f, 2.0f, 2.0f));
	chasis.setRotate(Eigen::AngleAxisf(0.0f, Eigen::Vector3f(0.0f, 1.0f, 0.0f)).toRotationMatrix());
	chasis.buildTexMatrix(1000.0f, 1000.0f);
	computeMomentsOfInertia();
}

void Vehicle::init() {
	chasis_shape.init();
	wheel_shape.init();
	ctex.init();
	wtex.init();
	for (auto &wheel : wheels) {
		wheel.initPQP();
	}
	chasis.initPQP();
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
	momenty = mass * (2.0f * 2.0f + 3.6f * 3.6f) / 12.0f;
	momentz = mass * (3.6f * 3.6f + 0.8f * 0.8f) / 12.0f;
}

void Vehicle::computeForces(const bool *keys, float dt) {
	Eigen::Vector3f f, fT, Fd, FdT, Fr, FrT, u, uT;
	Eigen::Vector3f Fa(0.0f, 0.0f, 0.0f), FaT(0.0f, 0.0f, 0.0f);

	// the direction vectors for forward and turning
	u(0) = sin(yaw + steerAngle);
	u(1) = 0.0f;
	u(2) = cos(yaw + steerAngle);
	uT(0) = sin(steerAngle);
	uT(1) = 0.0f;
	uT(2) = cos(steerAngle);
	u.normalize();
	uT.normalize();

	// Calculate the foward force
	float engineForce = 60000.0f;
	if (keys['w']) {
		Fa = u * engineForce;
		FaT = uT * engineForce;
		Fr = 0.0f * velocity;
		FrT = 0.0f * velocity;
	}
	else {
		if (keys['s']) {
			Fa = -u * 10000.0f;
			FaT = -uT * 10000.0f;
		}
		Fr = -125.0f * velocity;
		FrT = -125.0f * velocityT;
	}
	Fd = -50.00f * velocity * velocity.norm();
	FdT = -50.00f * velocityT * velocityT.norm();
	fT = FaT + FdT + FrT;
	f = Fa + Fd + Fr;
	
	// add up gravity vectors
	Eigen::Vector3f gravity = mass * 1.0f * Eigen::Vector3f(0.0f, 1.0f, 0.0f);
	for (int i = 0; i < 4; i++) {
		wforces[i] += gravity;
	}

	force = 2.0f * f + 4 * gravity;
	forceT = 2.0f * fT;
	
	torque = Eigen::Vector3f(0.5f, 0.0f, -0.8f).cross(fT) +
             Eigen::Vector3f(-0.5f, 0.0f, -0.8f).cross(fT) + 
             Eigen::Vector3f(-0.5f, 0.0f, -0.8).cross(wforces[0]) + 
             Eigen::Vector3f(0.5f, 0.0f, -0.8).cross(wforces[1]) + 
             Eigen::Vector3f(-0.5f, 0.0f, 0.8).cross(wforces[2]) + 
             Eigen::Vector3f(0.5f, 0.0f, 0.8).cross(wforces[3]);

	// reset the wheel forces to 0
	for (int i = 0; i < 4; i++) {
		wforces[i] = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
	}	
}

void Vehicle::update(const bool *keys, const Eigen::Vector2f &mouse, const std::vector<CollisionBox> &boxes, float dt) { 
	// Turning
	if (keys['a'] || keys['d']) {
		float maxTurn = (float)M_PI / 4;
		float turnFactor = (float)M_PI / 100;
		if (steerAngle < maxTurn && steerAngle > -maxTurn) {
		  steerAngle = keys['a'] ? steerAngle + turnFactor : steerAngle - turnFactor;
		}
	}
	else {
		steerAngle = 0.5f * steerAngle;
	}

	computeForces(keys, dt);

	// update velocity and position
	Eigen::Vector3f acceleration = force / mass;
	velocity += acceleration * dt;
	velocityT += (forceT / mass) * dt;
	position -= velocity * dt;

	// handle turning - uses torque to find amount to turn - broken into components
	float yawAcceleration = torque(1) / momenty / 6;
	float pitchAcceleration = torque(0) / momentx / 6;
	float rollAcceleration = torque(2) / momentz / 6;
	yawVelocity -= yawAcceleration * dt;
	pitchVelocity -= pitchAcceleration * dt;
	rollVelocity -= rollAcceleration * dt;
	yaw += yawVelocity * dt;
	pitch += pitchVelocity * dt;
	roll += rollVelocity * dt;
}

void Vehicle::draw(const bool *keys, MatrixStack &M, MatrixStack &V, MatrixStack &P, Program *prog, Light &light, bool isShadowPass1) {
	if (abs(velocity.norm()) > 0) tireSpin -= velocity.norm();

	// Set transforms and draw
	M.pushMatrix();
	M.translate(position + Eigen::Vector3f(0.0f, -1.0f, 0.0f));
	M.rotate(yaw, Eigen::Vector3f(0.0f, 1.0f, 0.0f));
	M.rotate(pitch, Eigen::Vector3f(1.0f, 0.0f, 0.0f));
	M.rotate(roll, Eigen::Vector3f(0.0f, 0.0f, 1.0f));
//	Eigen::Matrix3f Rs = Eigen::AngleAxisf(steerAngle, Eigen::Vector3f(0.0f, 1.0f, 0.0f)).toRotationMatrix();
//	Eigen::Matrix3f Rt = Eigen::AngleAxisf(tireSpin, Eigen::Vector3f(1.0f, 0.0f, 0.0f)).toRotationMatrix();

//	for (int i = 0; i < 2; i++) {  front wheels
//		Eigen::Matrix3f Rm = wheels[i].rotate;
//		wheels[i].rotate = Rs * Rt * Rm;
// 		wheels[i].draw(M, V, P, prog, light, isShadowPass1);
//		wheels[i].rotate = Rm;
//	}

//	for (int i = 2; i < 4; i++) {  rear wheels
//		Eigen::Matrix3f Rm = wheels[i].rotate;
//		wheels[i].rotate = Rt * Rm;
// 		wheels[i].draw(M, V, P, prog, light, isShadowPass1);
//		wheels[i].rotate = Rm;
//	}

	chasis.draw(M, V, P, prog, light, isShadowPass1);
	M.popMatrix();
}
