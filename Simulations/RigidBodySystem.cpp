#include "RigidBodySystem.h"

RigidBodySystem::RigidBodySystem(Vec3 p, Vec3 s, int m)
{
	position = p;
	size = s;
	mass = m;
	linear_velocity = Vec3(0.0f, 0.0f, 0.0f);
	orientation = Quat(Vec3(0.0f, 0.0f, 1.0f), 0.0f);
	angular_velocity = Vec3(0.0f, 0.0f, 0.0f);
}

void RigidBodySystem::setVelocity(Vec3 v)
{
	linear_velocity = v;
}

void RigidBodySystem::setOrientation(Quat o)
{
	orientation = o;
}

Vec3 RigidBodySystem::getPosition()
{
	return position;
}

Vec3 RigidBodySystem::getLinearVelocity()
{
	return linear_velocity;
}

Vec3 RigidBodySystem::getAngularVelocity()
{
	return angular_velocity;
}

void RigidBodySystem::applyForce(Vec3 loc, Vec3 force)
{

}