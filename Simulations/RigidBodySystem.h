#ifndef RIGIDBODYSYSTEM_h
#define RIGIDBODYSYSTEM_h
#include "Simulator.h"
#include <vector>

class RigidBodySystem {
public:
	RigidBodySystem(Vec3 p, Vec3 s, int m);
	void setVelocity(Vec3 v);
	void setOrientation(Quat o);
	void setMomentum(Vec3 mom);
	Vec3 getPosition();
	Vec3 getMomentum();
	Vec3 getLinearVelocity();
	Vec3 getAngularVelocity();
	void applyForce(Vec3 loc, Vec3 force);
	void simulateStep(float timeStep);
	Vec3 getWorldVelocity(Vec3 loc);
	Quat getOrientation();
	Vec3 getSize();
	Mat4 getWorldMatrix();
	int getMass();
	Mat4 getInertiaInv();
private:
	Vec3 position;
	Vec3 size;
	int mass;
	Vec3 linear_velocity;
	Vec3 angular_velocity;
	Quat orientation;
	Mat4 inertia_inv;
	Vec3 angular_momentum;
	vector<Vec3> forces;
	vector<Vec3> x_is;
};

#endif