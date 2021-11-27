#include "RigidBodySystem.h"

RigidBodySystem::RigidBodySystem(Vec3 p, Vec3 s, int m)
{
	position = p;
	size = s;
	mass = m;
	velocity = Vec3(0.0f, 0.0f, 0.0f);
	orientation = Quat(Vec3(0.0f, 0.0f, 1.0f), 0.0f);

}

void RigidBodySystem::setVelocity(Vec3 v)
{
	velocity = v;
}

void RigidBodySystem::setOrientation(Quat o)
{
	orientation = o;
}

Vec3 RigidBodySystem::getPosition()
{
	return position;
}