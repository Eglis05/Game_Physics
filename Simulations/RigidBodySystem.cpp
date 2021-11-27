#include "RigidBodySystem.h"

RigidBodySystem::RigidBodySystem(Vec3 p, Vec3 s, int m)
{
	position = p;
	size = s;
	mass = m;
	linear_velocity = Vec3(0.0f, 0.0f, 0.0f);
	orientation = Quat(Vec3(0.0f, 0.0f, 1.0f), 0.0f);
	angular_velocity = Vec3(0.0f, 0.0f, 0.0f);
	angular_momentum = Vec3(0.0f, 0.0f, 0.0f);
	float i_x = 1.0f / mass * (size.y * size.y + size.z * size.z);
	float i_y = 1.0f / mass * (size.x * size.x + size.z * size.z);
	float i_z = 1.0f / mass * (size.x * size.x + size.y * size.y);
	inertia_inv = Mat4(i_x, 0.0f, 0.0f, 0.0f, 0.0f, i_y, 0.0f, 0.0f, 0.0f, 0.0f, i_z, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	force = Vec3(0.0f, 0.0f, 0.0f);
	torque = Vec3(0.0f, 0.0f, 0.0f);
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

void RigidBodySystem::applyForce(Vec3 loc, Vec3 f)
{
	loc = loc - position;
	force += f;
	torque += cross(loc, f);
}

void RigidBodySystem::simulateStep(float timeStep)
{
	position += timeStep * linear_velocity;
	linear_velocity += timeStep * force / mass;

	orientation += timeStep / 2 * Quat(angular_velocity.x, angular_velocity.y, angular_velocity.z, 0) * orientation;

	angular_momentum += timeStep * torque;
	Mat4 rot_mat = orientation.getRotMat();
	Mat4 i_inv = rot_mat * inertia_inv;
	rot_mat.transpose();
	i_inv *= rot_mat;
	angular_velocity = i_inv * angular_momentum;
}