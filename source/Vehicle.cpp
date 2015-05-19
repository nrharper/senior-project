#include "../libraries/Vehicle.h"
#include "../libraries/Shape.h"
#include <iostream>

using namespace std;

Vehicle::Vehicle() :
	mass(1500.0f),
	yawVelocity(0.0f),
	pitchVelocity(0.0f),
	yaw((float)M_PI),
	pitch(0.0f),
	tireSpin(0.0f),
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
	ctex.setFilename("../materials/concrete.bmp");
	wtex.setFilename("../materials/concrete.bmp");

	WorldObject wheel0;
	wheel0.setShape(&wheel_shape);
	wheel0.setTexture(&wtex);
	wheel0.setTranslate(Eigen::Vector3f(-0.5f, 0.25f, -0.8f));
	wheel0.setScale(Eigen::Vector3f(0.75f, 0.75f, 0.25f));
	wheel0.setRotate(Eigen::AngleAxisf((float)M_PI / 2, Eigen::Vector3f(0.0f, 1.0f, 0.0f)).toRotationMatrix());
	wheels.push_back(wheel0);

	WorldObject wheel1;
	wheel1.setShape(&wheel_shape);
	wheel1.setTexture(&wtex);
	wheel1.setTranslate(Eigen::Vector3f(0.5f, 0.25f, -0.8f));
	wheel1.setScale(Eigen::Vector3f(0.75f, 0.75f, 0.25f));
	wheel1.setRotate(Eigen::AngleAxisf((float)M_PI * 3 / 2, Eigen::Vector3f(0.0f, 1.0f, 0.0f)).toRotationMatrix());
	wheels.push_back(wheel1);

	WorldObject wheel2;
	wheel2.setShape(&wheel_shape);
	wheel2.setTexture(&wtex);
	wheel2.setTranslate(Eigen::Vector3f(-0.5f, 0.25f, 0.8f));
	wheel2.setScale(Eigen::Vector3f(0.75f, 0.75f, 0.25f));
	wheel2.setRotate(Eigen::AngleAxisf((float)M_PI / 2, Eigen::Vector3f(0.0f, 1.0f, 0.0f)).toRotationMatrix());
	wheels.push_back(wheel2);

	WorldObject wheel3;
	wheel3.setShape(&wheel_shape);
	wheel3.setTexture(&wtex);
	wheel3.setTranslate(Eigen::Vector3f(0.5f, 0.25f, 0.8f));
	wheel3.setScale(Eigen::Vector3f(0.75f, 0.75f, 0.25f));
	wheel3.setRotate(Eigen::AngleAxisf((float)M_PI * 3 / 2, Eigen::Vector3f(0.0f, 1.0f, 0.0f)).toRotationMatrix());
	wheels.push_back(wheel3);

	Eigen::Vector3f wforce(0.0f, 0.0f, 0.0f);
	for (unsigned int i = 0; i < 4; i++) {
		wforces.push_back(wforce);
	}

	chasis.setShape(&chasis_shape);
	chasis.setTexture(&ctex);
	chasis.setTranslate(Eigen::Vector3f(0.0f, 0.25f, 0.0f));
	chasis.setScale(Eigen::Vector3f(1.0f, 0.4f, 1.0f));
	chasis.setRotate(Eigen::AngleAxisf(0.0f, Eigen::Vector3f(0.0f, 1.0f, 0.0f)).toRotationMatrix());
	computeMomentsOfInertia();
}

void Vehicle::init() {
	chasis_shape.init();
	wheel_shape.init();
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
	momenty = mass * (2.0f * 2.0f + 2.0f * 2.0f) / 12.0f;
}

void Vehicle::computeForces(const bool *keys, float dt) {
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
	
	force = 2.0f * f + wforces[0] + wforces[1] + wforces[2] + wforces[3];
	
	torque = Eigen::Vector3f(0.5f, 0.0f, -0.5f).cross(f) +
             Eigen::Vector3f(-0.5f, 0.0f, -0.5f).cross(f) + 
             Eigen::Vector3f(-0.5f, 0.0f, -0.5).cross(wforces[0]) + 
             Eigen::Vector3f(0.5f, 0.0f, -0.5).cross(wforces[1]) + 
             Eigen::Vector3f(-0.5f, 0.0f, 0.5).cross(wforces[2]) + 
             Eigen::Vector3f(0.5f, 0.0f, 0.5).cross(wforces[3]);
	//cout << torque(0) << " " << torque(1) << " " << torque(2) << endl;
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

	computeForces(keys, dt);

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
   //if (!checkCollisions(pos, vehicleAngle, boxes)) {
      position = Eigen::Vector3f(pos(0), pos(1), pos(2));
		//yaw = vehicleAngle;
   //}
//   else {
//      velocity << 0.0f, 0.0f, 0.0f;
//      yawVelocity = 0.0f;
//	}
}

void Vehicle::draw(const bool *keys, MatrixStack &M, MatrixStack &V, MatrixStack &P, Program *prog, Light &light, bool isShadowPass1) {
	if (abs(speed) > 0) tireSpin -= speed;

	// Set transforms and draw
	M.pushMatrix();
	M.translate(position + Eigen::Vector3f(0.0f, -1.0f, 0.0f));
	M.rotate(yaw, Eigen::Vector3f(0.0f, 1.0f, 0.0f));
	Eigen::Matrix3f Rs = Eigen::AngleAxisf(steerAngle, Eigen::Vector3f(0.0f, 1.0f, 0.0f)).toRotationMatrix();
	Eigen::Matrix3f Rt = Eigen::AngleAxisf(tireSpin, Eigen::Vector3f(1.0f, 0.0f, 0.0f)).toRotationMatrix();

	for (int i = 0; i < 2; i++) { // front wheels
		Eigen::Matrix3f Rm = wheels[i].rotate;
		wheels[i].rotate = Rs * Rt * Rm;
 		wheels[i].draw(M, V, P, prog, light, isShadowPass1);
		wheels[i].rotate = Rm;
	}

	for (int i = 2; i < 4; i++) { // rear wheels
		Eigen::Matrix3f Rm = wheels[i].rotate;
		wheels[i].rotate = Rt * Rm;
 		wheels[i].draw(M, V, P, prog, light, isShadowPass1);
		wheels[i].rotate = Rm;
	}

	chasis.draw(M, V, P, prog, light, isShadowPass1);
	M.popMatrix();
}
