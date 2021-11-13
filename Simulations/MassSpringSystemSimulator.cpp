#include "MassSpringSystemSimulator.h"

Mass::Mass(Vec3 p, Vec3 v, bool isfixed)
{
	position = p;
	velocity = v;
	isFixed = isfixed;
}

Vec3 Mass::getPosition()
{
	return position;
}

Vec3 Mass::getVelocity()
{
	return velocity;
}

bool Mass::getIsFixed()
{
	return isFixed;
}

void Mass::setPosition(Vec3 p)
{
	position = p;
}

void Mass::setVelocity(Vec3 v)
{
	velocity = v;
}

Spring::Spring(int mass_1, int mass_2, float initiallength)
{
	mass1 = mass_1;
	mass2 = mass_2;
	initialLength = initiallength;
}

int Spring::getMassOne()
{
	return mass1;
}

int Spring::getMassTwo()
{
	return mass2;
}

float Spring::getInitialLength()
{
	return initialLength;
}

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_iIntegrator = EULER;
	m_externalForce = Vec3();
	m_fMass = 1;
	m_fStiffness = 1;
	m_fDamping = 0;
	m_fSphereSize = 1;
}

void MassSpringSystemSimulator::setMass(float mass)
{
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	int nr_masses = getNumberOfMassPoints();
	masses.push_back(Mass(position, Velocity, isFixed));
	return nr_masses;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	springs.push_back(Spring(masspoint1, masspoint2, initialLength));
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return masses.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return masses[index].getPosition();
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return masses[index].getVelocity();
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	m_externalForce = force;
}
 
const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "Spring Test";
}

void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:
	case 1:
	case 2:
		TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_INT32, &m_fMass, "min=1");
		TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_INT32, &m_fStiffness, "min=1");
		TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_INT32, &m_fDamping, "min=1");
		TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INT32, &m_iIntegrator, "min=1 max=3 step=1");
		TwAddVarRW(DUC->g_pTweakBar, "Sphere Size", TW_TYPE_FLOAT, &m_fSphereSize, "min=0.01 step=0.01");
		break;
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	int p0;
	int p1;
	switch (m_iTestCase)
	{
	case 0:
		//setMass(10.0f);
		//setDampingFactor(0.0f);
		//setStiffness(40.0f);
		//applyExternalForce(Vec3(0, 0, 0));
		//p0 = addMassPoint(Vec3(0.0, 0.0f, 0), Vec3(-1.0, 0.0f, 0), false);
		//p1 = addMassPoint(Vec3(0.0, 2.0f, 0), Vec3(1.0, 0.0f, 0), false);
		//addSpring(p0, p1, 1.0);
		cout << "Teapot !\n";
		//for (int i = 0; i < 10; i++)
		//	simulateTimestep(0.005);;
		//cout << "Finished 10 timesteps" << endl;
		break;
	case 1:
		cout << "Random Object!\n";
		break;
	case 2:
		cout << "Triangle !\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
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

Vec3 MassSpringSystemSimulator::calculateForces(int index, int mass_nr)
{
	Vec3 total_forces = m_externalForce;
	Vec3 subtractions = getPositionOfMassPoint(springs[index].getMassOne()) - getPositionOfMassPoint(springs[index].getMassTwo());
	if (mass_nr == springs[index].getMassTwo())
		subtractions *= (-1);

	double l = norm(subtractions);
	total_forces -= m_fStiffness * (l - springs[index].getInitialLength()) / l * subtractions;
	return total_forces;
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame
	switch (m_iIntegrator)
	{// handling different cases
	case EULER:
		for (size_t i = 0; i < springs.size(); i++)
		{
			Vec3 total_force_one = calculateForces(i, springs[i].getMassOne());
			Vec3 total_force_two = calculateForces(i, springs[i].getMassTwo());
			int mass_one = springs[i].getMassOne();
			masses[mass_one].setPosition(getPositionOfMassPoint(mass_one) + timeStep * getVelocityOfMassPoint(mass_one));
			masses[mass_one].setVelocity(getVelocityOfMassPoint(mass_one) + timeStep / m_fMass * total_force_one);
			int mass_two = springs[i].getMassTwo();
			masses[mass_two].setPosition(getPositionOfMassPoint(mass_two) + timeStep * getVelocityOfMassPoint(mass_two));
			masses[mass_two].setVelocity(getVelocityOfMassPoint(mass_two) + timeStep / m_fMass * total_force_two);
		}
		break;
	case MIDPOINT:
		masses[0].setPosition(Vec3(2, 2, 2));
		break;
	default:
		break;
	}
}

void MassSpringSystemSimulator::drawSomeRandomObjects()
{
	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
	std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);
	for (int i = 0; i < getNumberOfMassPoints(); i++)
	{
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
		DUC->drawSphere(Vec3(randPos(eng), randPos(eng), randPos(eng)), Vec3(m_fSphereSize, m_fSphereSize, m_fSphereSize));
	}
}

void MassSpringSystemSimulator::drawMovableTeapot()
{
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(0.97, 0.86, 1));
	//DUC->drawTeapot(m_vfMovableObjectPos, m_vfRotate, Vec3(0.5, 0.5, 0.5));
}

void MassSpringSystemSimulator::drawTriangle()
{
	DUC->DrawTriangleUsingShaders();
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	switch (m_iTestCase)
	{
	case 0: drawMovableTeapot();break;
	case 1: drawSomeRandomObjects();break;
	case 2: drawTriangle();break;
	}
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
