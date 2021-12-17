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
	//float i_x = 1.0f / (mass * (size.y * size.y / 4 + size.z * size.z / 4));
	//float i_y = 1.0f / (mass * (size.x * size.x / 4 + size.z * size.z / 4));
	//float i_z = 1.0f / (mass * (size.x * size.x / 4 + size.y * size.y / 4));
	float i_x = 12.0f / (mass * (size.y * size.y + size.z * size.z));
	float i_y = 12.0f / (mass * (size.x * size.x + size.z * size.z));
	float i_z = 12.0f / (mass * (size.x * size.x + size.y * size.y));
	inertia_inv = Mat4(i_x, 0.0f, 0.0f, 0.0f, 0.0f, i_y, 0.0f, 0.0f, 0.0f, 0.0f, i_z, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void RigidBodySystem::setVelocity(Vec3 v)
{
	linear_velocity = v;
}

void RigidBodySystem::setMomentum(Vec3 mom)
{
	angular_momentum = mom;
}

void RigidBodySystem::setOrientation(Quat o)
{
	orientation = o;
}

Vec3 RigidBodySystem::getPosition()
{
	return position;
}

Quat RigidBodySystem::getOrientation()
{
	return orientation;
}

Vec3 RigidBodySystem::getMomentum()
{
	return angular_momentum;
}

Vec3 RigidBodySystem::getSize()
{
	return size;
}

Vec3 RigidBodySystem::getLinearVelocity()
{
	return linear_velocity;
}

Vec3 RigidBodySystem::getAngularVelocity()
{
	return angular_velocity;
}

int RigidBodySystem::getMass()
{
	return mass;
}

Mat4 RigidBodySystem::getInertiaInv()
{
	Mat4 rot_mat = orientation.getRotMat();
	Mat4 result = rot_mat * inertia_inv;
	rot_mat.transpose();
	return result * rot_mat;
}

Vec3 RigidBodySystem::getWorldVelocity(Vec3 loc)
{
	return linear_velocity + cross(angular_velocity, loc);
}

void RigidBodySystem::applyForce(Vec3 loc, Vec3 f)
{
	forces.push_back(f);
	x_is.push_back(loc);
}

void RigidBodySystem::simulateStep(float timeStep)
{
	
	Vec3 force = Vec3(0.0f, 0.0f, 0.0f);
	Vec3 torque = Vec3(0.0f, 0.0f, 0.0f);
	for (int i = 0; i < forces.size(); i++)
	{
		Vec3 x_ihat = (x_is[i] - position).getAbsolutes();
		if (1)//x_ihat.x <= size.x / 2 && x_ihat.y <= size.y / 2 && x_ihat.z <= size.z / 2) 
		{
			//not sure if i should check if force is in the box
			force += forces[i];
			torque += cross(x_is[i] - position, forces[i]);
		}
		//cout << "cross: ";
		//cout << x_ihat << endl;
	}
	//cout << "force:" << force << endl;
	//cout << "torque:" << torque << endl;
	//cout << "timeStep:" << timeStep << endl;
	position += timeStep * linear_velocity;
	//cout << "pos:" << position << endl;
	linear_velocity += timeStep * force / mass;
	//cout << "lin_v:" << linear_velocity << endl;
	orientation += timeStep / 2 * Quat(angular_velocity.x, angular_velocity.y, angular_velocity.z, 0) * orientation;
	orientation = orientation.unit();
	//cout << "ori:" << orientation << endl;
	angular_momentum += timeStep * torque;
	//cout << "ang_m:" << angular_momentum << endl;
	Mat4 rot_mat = orientation.getRotMat();
	//cout << "rot mat:" << rot_mat << endl;
	Mat4 i_inv = rot_mat * inertia_inv;
	rot_mat.transpose();
	i_inv *= rot_mat;
	//cout << "I-1:" << i_inv << endl;
	angular_velocity = i_inv.transformVector(angular_momentum);
	//cout << "ang_v:" << angular_velocity << endl;
}

Mat4 RigidBodySystem::getWorldMatrix()
{
	Mat4 A_world = Mat4(1, 0, 0, position.x, 0, 1, 0, position.y, 0, 0, 1, position.z, 0, 0, 0, 1);
	A_world = orientation.getRotMat() * A_world;
	A_world = Mat4(size.x, 0, 0, 0, 0, size.y, 0, 0, 0, 0, size.z, 0, 0, 0, 0, 1) * A_world;
	return A_world;
}