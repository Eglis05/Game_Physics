#include "RigidBodySystemSimulator.h"
//#include "RigidBodySystem.h"

const char* RigidBodySystemSimulator::getTestCasesStr() {
	return "Demo 2: EULER, Demo 4: Complex, Demo 3: MIDPOINT";
}

const int EULER_DEMO = 0;
const int COMPLEX_DEMO = 1;
const int MIDPOINT_DEMO = 2;


RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_iTestCase = TESTCASEUSEDTORUNTEST;
	m_externalForce = Vec3(0, -1.0f, 0);
	m_iIntegrator = 0;
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
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwEnumVal integratorsEV[] = { { 0, "Euler" }, { 1, "Midpoint" } };
	TwType twIntegrator = TwDefineEnum("Integrator", integratorsEV, 2);

	switch (m_iTestCase)
	{
	case EULER_DEMO: //Demo 2: Euler
		//set integrator to EULER
		m_iIntegrator = 0;
		//TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=0.01 step=0.1");
		//TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=0 step=0.1");
		//TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0 max=1");
		//TwAddVarRW(DUC->g_pTweakBar, "Sphere Size", TW_TYPE_FLOAT, &m_fSphereSize, "min=0.01 max=0.5 step=0.01");
		break;
	case COMPLEX_DEMO: //Demo 4: Complex Scene
		//TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=0.01 step=0.1");
		//TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=0 step=0.1");
		//TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0 max=1");
		TwAddVarRW(DUC->g_pTweakBar, "Integrator", twIntegrator, &m_iIntegrator, "");
		//TwAddVarRW(DUC->g_pTweakBar, "Sphere Size", TW_TYPE_FLOAT, &m_fSphereSize, "min=0.01 max=0.5 step=0.01");
		//TwAddVarRW(DUC->g_pTweakBar, "Bounce", TW_TYPE_FLOAT, &m_fBounce, "min=0 max=0.9 step=0.1");
		break;
	case MIDPOINT_DEMO: //Demo 3: Midpoint
		//set integrator to MIDPOINT
		//m_iIntegrator = MIDPOINT;
		//TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=0.01 step=0.1");
		//TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=0 step=0.1");
		//TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0 max=1");
		//TwAddVarRW(DUC->g_pTweakBar, "Sphere Size", TW_TYPE_FLOAT, &m_fSphereSize, "min=0.01 max=0.5 step=0.01");
		break;
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	reset();
	int p0;
	int p1;
	switch (m_iTestCase)
	{
	case EULER_DEMO:
		cout << "Demo 2: EULER-Simulation\n";
		//m_iIntegrator = EULER;
		//setupSimpleScene();
		break;
	case COMPLEX_DEMO:
		cout << "Demo 4: Complex Scene\n";
		//m_iIntegrator = EULER;
		//setupComplexScene();
		break;
	case MIDPOINT_DEMO:
		cout << "Demo 3: MIDPOINT-Simulation\n";
		//m_iIntegrator = MIDPOINT;
		//setupSimpleScene();
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

/*
void RigidBodySystemSimulator::setupComplexScene()
{
	int p0 = addMassPoint(Vec3(0.0f, 0, 0), Vec3(0, 0, 0), false); //mass 0
	int p1 = addMassPoint(Vec3(1.0f, 0, 0), Vec3(0, 0, 0), false); //mass 1 etc..
	int p2 = addMassPoint(Vec3(1.0f, 0, 1.0f), Vec3(0, 0, 0), false);
	int p3 = addMassPoint(Vec3(0, 0, 1.0f), Vec3(0, 0, 0), false);

	int p4 = addMassPoint(Vec3(0.0f, 1.0f, 0), Vec3(0, 0, 0), false);
	int p5 = addMassPoint(Vec3(1.0f, 1.0f, 0), Vec3(0, 0, 0), false);
	int p6 = addMassPoint(Vec3(1.0f, 1.0f, 1.0f), Vec3(0, 0, 0), false);
	int p7 = addMassPoint(Vec3(0, 1.0f, 1.0f), Vec3(0, 0, 0), false);

	m_fMass = 1;
	m_fStiffness = 1;

	//lower square
	addSpring(p0, p1, 1.0);
	addSpring(p1, p2, 1.0);
	addSpring(p2, p3, 1.0);
	addSpring(p3, p0, 1.0);
	//upper square
	addSpring(p4, p5, 1.0);
	addSpring(p5, p6, 1.0);
	addSpring(p6, p7, 1.0);
	addSpring(p7, p4, 1.0);
	//vertical Edges
	addSpring(p0, p4, 1.0);
	addSpring(p1, p5, 1.0);
	addSpring(p2, p6, 1.0);
	addSpring(p3, p7, 1.0);
	//add vertical Cross-edges
	addSpring(p0, p5, 1.0);
	addSpring(p0, p7, 1.0);
	addSpring(p1, p4, 1.0);
	addSpring(p1, p6, 1.0);
	addSpring(p2, p5, 1.0);
	addSpring(p2, p7, 1.0);
	addSpring(p3, p4, 1.0);
	addSpring(p3, p6, 1.0);
	//add horizontal Cross-edges
	addSpring(p0, p2, 1.0);
	addSpring(p1, p3, 1.0);
	addSpring(p4, p6, 1.0);
	addSpring(p5, p7, 1.0);
}

void RigidBodySystemSimulator::setupSimpleScene()
{
	int p0 = addMassPoint(Vec3(0.0, 0.0f, 0), Vec3(-1.0f, 0, 0), false); //mass 0
	int p1 = addMassPoint(Vec3(0.0, 2.0f, 0), Vec3(1.0f, 0, 0), false); //mass 1

	m_fMass = 10;
	m_fStiffness = 40;


	addSpring(p0, p1, 1.0); //Spring Stiffness
}
*/

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
/*
Vec3 RigidBodySystemSimulator::calculateForces(int index, int mass_nr)
{
	Vec3 total_forces = Vec3();
	if (m_iTestCase == COMPLEX_DEMO)
		total_forces = m_externalForce;
	Vec3 subtractions = getPositionOfMassPoint(springs[index].getMassOne()) - getPositionOfMassPoint(springs[index].getMassTwo());
	if (mass_nr == springs[index].getMassTwo())
		subtractions *= (-1);

	double l = norm(subtractions);
	total_forces -= m_fStiffness * (l - springs[index].getInitialLength()) / l * subtractions;
	return total_forces;
}
*/

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	for (int i = 0; i < getNumberOfRigidBodies(); i++)
	{
		m_RigidBodySystem[i].simulateStep(timeStep);
	}
	/*
	//Check for collision with floor only for Demo 4
	if (m_iTestCase == COMPLEX_DEMO)
	{
		for (int i = 0; i < masses.size(); i++)
		{
			Vec3 mass_pos = masses[i].getPosition();
			Vec3 mass_vel = masses[i].getVelocity();
			//simple collision detection and management
			if (mass_pos.y <= (-1.0 + m_fSphereSize))
			{
				mass_pos.y = -1.0 + m_fSphereSize;
				mass_vel.y = mass_vel.y * m_fBounce * -1;
				masses[i].setPosition(mass_pos);
				masses[i].setVelocity(mass_vel);
			}
		}
	}
	*/
}

/**
 * This function draws the system class springs
 **/
void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(0.97, 0.86, 1));
	/*
	for (int i = 0; i < springs.size(); i++)
	{
		DUC->beginLine();
		DUC->drawLine(getPositionOfMassPoint(springs[i].getMassOne()), Vec3(1, 0, 0), getPositionOfMassPoint(springs[i].getMassTwo()), Vec3(1, 0, 0));
		DUC->endLine();
	}
	for (int i = 0; i < masses.size(); i++)
	{
		DUC->drawSphere(getPositionOfMassPoint(i), Vec3(m_fSphereSize, m_fSphereSize, m_fSphereSize));
	}
	*/
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
