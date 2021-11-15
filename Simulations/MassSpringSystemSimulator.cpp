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
	m_iTestCase = 0;
	m_iIntegrator = EULER;
	m_externalForce = Vec3(0,-1.0f,0);
	m_fMass = 1.0f;
	m_fStiffness = 0.5f;
	m_fDamping = 0;
	m_fSphereSize = 0.05f;
	m_fBounce = 0.2f;
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
	return "Demo 2: EULER, Demo 4, Demo 3: MIDPOINT";
}

void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	//Clear all masses and springs
	masses.clear();
	springs.clear();
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0: //Demo 2: Euler
		//set integrator to EULER
		m_iIntegrator = EULER;
		TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=0.01 step=0.1");
		TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=0 step=0.1");
		//TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0 max=1");
		TwAddVarRW(DUC->g_pTweakBar, "Sphere Size", TW_TYPE_FLOAT, &m_fSphereSize, "min=0.01 max=0.5 step=0.01");	
		break;
	case 1: //Demo 4: Complex Scene
		TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=0.01 step=0.1");
		TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=0 step=0.1");
		//TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0 max=1");
		TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INT32, &m_iIntegrator, "min=0 max=2 step=2");
		TwAddVarRW(DUC->g_pTweakBar, "Sphere Size", TW_TYPE_FLOAT, &m_fSphereSize, "min=0.01 max=0.5 step=0.01");
		TwAddVarRW(DUC->g_pTweakBar, "Bounce", TW_TYPE_FLOAT, &m_fBounce, "min=0 max=0.9 step=0.1");
		break;
	case 2: //Demo 3: Midpoint
		//set integrator to MIDPOINT
		m_iIntegrator = MIDPOINT;
		TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=0.01 step=0.1");
		TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=0 step=0.1");
		//TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0 max=1");
		TwAddVarRW(DUC->g_pTweakBar, "Sphere Size", TW_TYPE_FLOAT, &m_fSphereSize, "min=0.01 max=0.5 step=0.01");
		break;
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	reset();
	int p0;
	int p1;
	switch (m_iTestCase)
	{
	case 0:
		cout << "Demo 2: EULER-Simulation\n";
		m_iIntegrator = 0;
		setupDemoScene();
		break;
	case 1:
		cout << "Demo 4: Complex Scene\n";
		m_iIntegrator = 0;
		setupScene();
		break;
	case 2:
		cout << "Demo 3: MIDPOINT-Simulation\n";
		m_iIntegrator = 2;
		setupDemoScene();
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void MassSpringSystemSimulator::setupScene()
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

void MassSpringSystemSimulator::setupDemoScene()
{
	int p0 = addMassPoint(Vec3(0.0, 0.0f, 0), Vec3(-1.0f, 0, 0), false); //mass 0
	int p1 = addMassPoint(Vec3(0.0, 2.0f, 0), Vec3(1.0f, 0, 0), false); //mass 1

	m_fMass = 10;
	m_fStiffness = 40;
	

	addSpring(p0, p1, 1.0); //Spring Stiffness
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
	Vec3 total_forces = Vec3();
	if(m_iTestCase == 1) 
		total_forces = m_externalForce;
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
		for (size_t i = 0; i < springs.size(); i++)
		{
			//calculate half and quarter timeStep h
			float h_timeStep = timeStep / 2.0f;
			float q_timeStep = timeStep / 4.0f;
			int mass_one = springs[i].getMassOne();
			int mass_two = springs[i].getMassTwo();

			//saved for later use
			Vec3 mass_one_pos = getPositionOfMassPoint(mass_one);
			Vec3 mass_one_vel = getVelocityOfMassPoint(mass_one);
			Vec3 mass_two_pos = getPositionOfMassPoint(mass_two);
			Vec3 mass_two_vel = getVelocityOfMassPoint(mass_two);

			//calculate and save quarter pos
			masses[mass_one].setPosition(mass_one_pos + q_timeStep * mass_one_vel);
			masses[mass_two].setPosition(mass_two_pos + q_timeStep * mass_two_vel);
			//calculate force with quarter pos
			Vec3 q_total_force_one = calculateForces(i, mass_one);
			Vec3 q_total_force_two = calculateForces(i, mass_two);

			//calculate half pos
			masses[mass_one].setPosition(mass_one_pos + h_timeStep * mass_one_vel);
			masses[mass_two].setPosition(mass_two_pos + h_timeStep * mass_two_vel);
			//recalculate force with half pos
			Vec3 h_total_force_one = calculateForces(i, mass_one);
			Vec3 h_total_force_two = calculateForces(i, mass_two);

			// calculate velocity with quarter pos force
			masses[mass_one].setVelocity(mass_one_vel + h_timeStep / m_fMass * q_total_force_one);
			masses[mass_two].setVelocity(mass_two_vel + h_timeStep / m_fMass * q_total_force_two);
			//calculate final pos and velocity
			masses[mass_one].setPosition(mass_one_pos + timeStep * getVelocityOfMassPoint(mass_one));
			masses[mass_one].setVelocity(mass_one_vel + timeStep / m_fMass * h_total_force_one);
			masses[mass_two].setPosition(mass_two_pos + timeStep * getVelocityOfMassPoint(mass_two));
			masses[mass_two].setVelocity(mass_two_vel + timeStep / m_fMass * h_total_force_two);
		}

		break;
	default:
		break;
	}

	//Check for collision with floor only for Demo 4
	if (m_iTestCase == 1)
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
}

/**Original Drawing Functions
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
	DUC->drawTeapot(Vec3(0,0,0), Vec3(0,0,0), Vec3(0.5, 0.5, 0.5));
}

void MassSpringSystemSimulator::drawTriangle()
{
	DUC->DrawTriangleUsingShaders();
}
*/

void MassSpringSystemSimulator::drawSpringObjects()
{
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(0.97, 0.86, 1));
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
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawSpringObjects();
	//switch (m_iTestCase)
	//{
	//case 0: drawSpringObjects(); break;
	//case 1: drawSpringObjects(); break;
	//case 2: drawSpringObjects(); break;
	//}
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
