#include "RigidBodySystemSimulator.h"
#include <Windows.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <cstdlib>
#include "collisionDetect.h"
using namespace std;

const char* RigidBodySystemSimulator::getTestCasesStr() {
	return "Demo 1: Simple One Step, Demo 2: Body Simulation, Demo 3: Two Body Collision, Demo 4: Complex";
}

const int ONESTEP_DEMO = 0;
const int BODYSIMULATION_DEMO = 1;
const int COLLISION_DEMO = 2;
const int COMPLEX_DEMO = 3;


RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_iTestCase = ONESTEP_DEMO;
	m_externalForce = Vec3(0, 0, 0);
	c_impulse = 0;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	m_RigidBodySystem[i].setVelocity(velocity);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	m_RigidBodySystem[i].setOrientation(orientation);
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	m_RigidBodySystem.push_back(RigidBodySystem(position, size, mass));
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return m_RigidBodySystem.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return m_RigidBodySystem[i].getPosition();
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return m_RigidBodySystem[i].getLinearVelocity();
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return m_RigidBodySystem[i].getAngularVelocity();
}

Vec3 RigidBodySystemSimulator::getWorldVelocityOfRigidBody(int i, Vec3 loc)
{
	return m_RigidBodySystem[i].getWorldVelocity(loc);
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	m_RigidBodySystem[i].applyForce(loc, force);
}

void RigidBodySystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	//Clear all bodies
	m_RigidBodySystem.clear();
	m_externalForce = Vec3(0, 0, 0);
	c_impulse = 0;
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;

	switch (m_iTestCase)
	{
	case ONESTEP_DEMO:
		setupOneStep();
		break;
	case BODYSIMULATION_DEMO:
		setupBodySimulation();
		break;
	case COLLISION_DEMO:
		setupCollision();
		break;
	case COMPLEX_DEMO:
		setupComplex();
		break;
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	reset();
	cout << "Demo " << m_iTestCase + 1 << endl;
	switch (m_iTestCase)
	{
	case ONESTEP_DEMO:
		setupOneStep();
		break;
	case BODYSIMULATION_DEMO:
		setupBodySimulation();
		break;
	case COLLISION_DEMO:
		setupCollision();
		break;
	case COMPLEX_DEMO:
		setupComplex();
		break;
	}
}

void RigidBodySystemSimulator::setupOneStep()
{
	addRigidBody(Vec3(0.0f, 0.0f, 0.0f), Vec3(1.0f, 0.6f, 0.5f), 2.0f);
	setOrientationOf(0, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.5));
	applyForceOnBody(0, Vec3(0.3f, 0.5f, 0.25f), Vec3(1.0f, 1.0f, 0.0f));
	simulateTimestep(2);
	cout << "Linear Velocity: " << getLinearVelocityOfRigidBody(0) << endl;
	cout << "Angular Velocity: " << getAngularVelocityOfRigidBody(0) << endl;
	Vec3 x = Vec3(0.3, 0.5, 0.25);
	cout << "Wolrd Space Velocity of Point " << x << " is: " << getWorldVelocityOfRigidBody(0, x) << endl;
	this_thread::sleep_for(chrono::seconds(10));
}

void RigidBodySystemSimulator::setupBodySimulation()
{
	addRigidBody(Vec3(0.0f, 0.0f, 0.0f), Vec3(1.0f, 0.6f, 0.5f), 2.0f);
	setOrientationOf(0, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.5));
	applyForceOnBody(0, Vec3(0.3f, 0.5f, 0.25f), Vec3(1.0f, 1.0f, 0.0f));
	for (int i = 0; i < 200; i++)
		simulateTimestep(0.01);
	cout << "Run for 200 steps of 0.01" << endl;
	cout << "Linear Velocity: " << getLinearVelocityOfRigidBody(0) << endl;
	cout << "Angular Velocity: " << getAngularVelocityOfRigidBody(0) << endl;
	Vec3 x = Vec3(0.3, 0.5, 0.25);
	cout << "Wolrd Space Velocity of Point " << x << " is: " << getWorldVelocityOfRigidBody(0, x) << endl;
	this_thread::sleep_for(chrono::seconds(1));
}

void RigidBodySystemSimulator::setupCollision()
{
	testCheckCollision(0);
	testCheckCollision(1);
	testCheckCollision(2);
}

void RigidBodySystemSimulator::setupComplex()
{

}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.001f;
		inputWorld = inputWorld * inputScale;
		//m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
	}
	//else {
		//m_vfMovableObjectFinalPos = m_vfMovableObjectPos;
	//}
}



void RigidBodySystemSimulator::checkCollision(RigidBodySystem A, RigidBodySystem B)
{
	Mat4 A_world = A.getWorldMatrix();
	Mat4 B_world = B.getWorldMatrix();
	CollisionInfo info = checkCollisionSAT(A_world, B_world);
	if (info.isValid)
	{
		Vec3 v_rel = A.getWorldVelocity(info.collisionPointWorld) - B.getWorldVelocity(info.collisionPointWorld);
		if (dot(v_rel, info.normalWorld) < 0)
		{
			Vec3 x_a = info.collisionPointWorld - A.getPosition();
			Vec3 x_b = info.collisionPointWorld - B.getPosition();
			float impulse = -(1 + c_impulse) * dot(v_rel, info.normalWorld) /
				(1.0 / A.getMass() + 1.0 / B.getMass() + dot(cross(A.getInertiaInv() * cross(x_a, info.normalWorld), x_a) +
					cross(B.getInertiaInv() * cross(x_b, info.normalWorld), x_b), info.normalWorld));

			A.setVelocity(A.getLinearVelocity() + impulse * info.normalWorld / A.getMass());
			B.setVelocity(B.getLinearVelocity() + impulse * info.normalWorld / B.getMass());

			A.setMomentum(A.getMomentum() + cross(x_a, impulse * info.normalWorld));
			B.setMomentum(B.getMomentum() + cross(x_b, impulse * info.normalWorld));
		}
	}
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	for (int i = 0; i < getNumberOfRigidBodies(); i++)
	{
		m_RigidBodySystem[i].simulateStep(timeStep);
	}

	for (int i = 0; i < getNumberOfRigidBodies() - 1; i++)
	{
		for (int j = i + 1; j < getNumberOfRigidBodies(); j++)
		{
			checkCollision(m_RigidBodySystem[i], m_RigidBodySystem[j]);
		}
	}
}

/**
 * This function draws the system class springs
 **/
void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 2000, Vec3(0.5, 0.5, 0.5));
	for (int i = 0; i < m_RigidBodySystem.size(); i++)
	{
		DUC->drawRigidBody(m_RigidBodySystem[i].getWorldMatrix());
	}
}

/**
 * Mouse interactions are not working
 **/
void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

/**
 * Mouse interactions are not working
 **/
void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
