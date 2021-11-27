#ifndef RIGIDBODYSYSTEM_h
#define RIGIDBODYSYSTEM_h
#include "Simulator.h"

class RigidBodySystem {
public:
	RigidBodySystem(Vec3 p, Vec3 s, int m);
	void setVelocity(Vec3 v);
	void setOrientation(Quat o);
	Vec3 getPosition();
	Vec3 getLinearVelocity();
	Vec3 getAngularVelocity();
	void applyForce(Vec3 loc, Vec3 force);
	void simulateStep(float timeStep);
private:
	Vec3 position;
	Vec3 size;
	int mass;
	Vec3 linear_velocity;
	Vec3 angular_velocity;
	Quat orientation;
	Mat4 inertia_inv;
	Vec3 angular_momentum;
	Vec3 force;
	Vec3 torque;
};

#endif