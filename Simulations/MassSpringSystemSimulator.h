#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"
#include <vector>

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change

class Mass {
public:
	Mass(Vec3 p, Vec3 v, bool isfixed);
	Vec3 getPosition();
	Vec3 getVelocity();
	bool getIsFixed();
	void setPosition(Vec3 p);
	void setVelocity(Vec3 v);
private:
	Vec3 position;
	Vec3 velocity;
	bool isFixed;
};

class Spring {
public:
	Spring(int mass_1, int mass_2, float initiallength);
	int getMassOne();
	int getMassTwo();
	float getInitialLength();
private:
	int mass1;
	int mass2;
	float initialLength;
};


class MassSpringSystemSimulator:public Simulator{
public:
	// Construtors
	MassSpringSystemSimulator();
	
	// UI Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);

	// Extra Functions
	Vec3 calculateForces(int index, int mass_nr);
	//void drawSomeRandomObjects();
	//void drawMovableTeapot();
	//void drawTriangle();
	void drawSpringObjects();
	void setupScene();
	void setupDemoScene();
	
	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

private:
	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;

	// Extra Attributes
	vector<Mass> masses;
	vector<Spring> springs;
	float m_fSphereSize;
	float m_fBounce;

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};
#endif