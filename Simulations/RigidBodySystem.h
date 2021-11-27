#ifndef RIGIDBODYSYSTEM_h
#define RIGIDBODYSYSTEM_h
#include "Simulator.h"

class RigidBodySystem {
public:
	RigidBodySystem(Vec3 p, Vec3 s, int m);
	void setVelocity(Vec3 v);
	void setOrientation(Quat o);
	Vec3 getPosition();
private:
	Vec3 position;
	Vec3 size;
	int mass;
	Vec3 velocity;
	Quat orientation;
};

#endif