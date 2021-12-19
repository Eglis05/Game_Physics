#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;

Grid::Grid() {
}

void Grid::InitializeZero(int m_new, int n_new) {
	m = m_new;
	n = n_new;
	vector < vector < float > > zeros_v(m, std::vector<float>(n));
	grid_T = zeros_v;
	dx = 1.0 / m;
	dy = 1.0 / n;
}

float Grid::getValue(int i, int j) {
	i = min(max(i, 0), m-1);
	j = min(max(j, 0), n-1);
	//cout << i << " " << j << endl;
	return grid_T[i][j];
}

void Grid::setValue(int i, int j, float new_t) {
	grid_T[i][j] = new_t;
}

float Grid::getDX() {
	return dx;
}

float Grid::getDY() {
	return dy;
}


DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	// to be implemented
	m_Grid_m = 16;
	m_Grid_n = 16;
	m_alpha = 0.5;
}

const char * DiffusionSimulator::getTestCasesStr(){
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset(){
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	// to be implemented
	TwAddVarRW(DUC->g_pTweakBar, "Grid M", TW_TYPE_INT32, &m_Grid_m, "min = 1");
	TwAddVarRW(DUC->g_pTweakBar, "Grid N", TW_TYPE_INT32, &m_Grid_n, "min = 1");
	TwAddVarRW(DUC->g_pTweakBar, "Alpha", TW_TYPE_FLOAT, &m_alpha, "step=0.001  min=0.001");
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	//
	//to be implemented
	//
	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit solver!\n";
		break;
	case 1:
		cout << "Implicit solver!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
	T = new Grid();
	setUpScene();
}

void DiffusionSimulator::setUpScene() {
	T->InitializeZero(m_Grid_m, m_Grid_n);
	for (int i = m_Grid_m / 2; i < m_Grid_m - 1; i++) {
		for (int j = m_Grid_n / 2; j < m_Grid_n - 1; j++) {
			T->setValue(i, j, 1);
		}
	}
}

Grid* DiffusionSimulator::diffuseTemperatureExplicit(float timeStep) {//add your own parameters
	Grid* newT = new Grid();
	// to be implemented
	//make sure that the temperature in boundary cells stays zero
	newT->InitializeZero(m_Grid_m, m_Grid_n);
	for (int i = 1; i < m_Grid_m - 1; i++) {
		for (int j = 1; j < m_Grid_n - 1; j++) {
			float d_d_x = (T->getValue(i + 1, j) - 2 * T->getValue(i, j) + T->getValue(i - 1, j)) / (4 * T->getDX() * T->getDX());
			float d_d_y = (T->getValue(i, j + 1) - 2 * T->getValue(i, j) + T->getValue(i, j - 1)) / (4 * T->getDY() * T->getDY());
			float T_ij = timeStep * m_alpha * (d_d_x + d_d_y) + T->getValue(i, j);
			newT->setValue(i, j, T_ij);
		}
	}
	return newT;
}

void setupB(std::vector<Real>& b, double factor, int Grid_m, int Grid_n, float timeStep, float dx, float dy, Grid* T) {//add your own parameters
	// to be implemented
	//set vector B[sizeX*sizeY]
	for (int i = 0; i < Grid_m; i++) {
		for (int j = 0; j < Grid_n; j++) {
			float first_t  = T->getValue(i, j) / timeStep;
			float second_x = 1.0 / (dx * dx) * (T->getValue(i + 1, j) - 2 * T->getValue(i, j) + T->getValue(i - 1, j));
			float third_y  = 1.0 / (dy * dy) * (T->getValue(i, j + 1) - 2 * T->getValue(i, j) + T->getValue(i, j - 1));
			b.at(i * Grid_n + j) = first_t + factor / 2 * (second_x + third_y);
		}
	}
}

void fillT(std::vector<Real>& x, int Grid_m, int Grid_n, Grid* T) {//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero
	for (int i = 0; i < Grid_m; i++) {
		for (int j = 0; j < Grid_n; j++) {
			if (i == 0 || i == Grid_m - 1 || j == 0 || j == Grid_n - 1)
				T->setValue(i, j, 0);
			else
				T->setValue(i, j, x.at(i * Grid_n + j));
		}
	}
}

void setupA(SparseMatrix<Real>& A, double factor, int Grid_m, int Grid_n, float timeStep, float dx, float dy) {//add your own parameters
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	for (int i = 0; i < Grid_m; i++) {
		for (int j = 0; j < Grid_n; j++) {
			if (i == 0 || i == Grid_m - 1 || j == 0 || j == Grid_n - 1) {
				int diag_pos = i * Grid_n + j;
				A.set_element(diag_pos, diag_pos, 1);
			}
			else {
				int pos_ij = i * Grid_n + j;
				A.set_element(pos_ij, pos_ij, 1.0 / timeStep + factor * (1.0 / (dx * dx) + 1.0 / (dy * dy)));
				A.set_element(pos_ij, pos_ij + 1 * Grid_n, -factor / (2 * dx * dx));
				A.set_element(pos_ij, pos_ij - 1 * Grid_n, -factor / (2 * dx * dx));
				A.set_element(pos_ij, pos_ij + 1, -factor / (2 * dy * dy));
				A.set_element(pos_ij, pos_ij - 1, -factor / (2 * dy * dy));
			}
		}
	}
}


void DiffusionSimulator::diffuseTemperatureImplicit(float timeStep) {//add your own parameters
	// solve A T = b
	// to be implemented
	const int N = m_Grid_m * m_Grid_n;//N = sizeX*sizeY*sizeZ
	SparseMatrix<Real> *A = new SparseMatrix<Real> (N);
	std::vector<Real> *b = new std::vector<Real>(N);

	setupA(*A, m_alpha, m_Grid_m, m_Grid_n, timeStep, T->getDX(), T->getDY());
	setupB(*b, m_alpha, m_Grid_m, m_Grid_n, timeStep, T->getDX(), T->getDY(), T);

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N);
	for (int j = 0; j < N; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(*A, *b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	// x contains the new temperature values
	fillT(x, m_Grid_m, m_Grid_n, T);//copy x to T
}



void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// to be implemented
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		T = diffuseTemperatureExplicit(timeStep);
		break;
	case 1:
		diffuseTemperatureImplicit(timeStep);
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	// to be implemented
	//visualization
	//DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, colors[i % 3]);
	
	for (int i = 0; i < m_Grid_m; i++) {
		for (int j = 0; j < m_Grid_n; j++) {
			float color = T->getValue(i, j);
			DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, Vec3(color, color, color));//0.6 * Vec3(0.97, 0.86, 1));
			Vec3 pos = i * Vec3(1.0 / m_Grid_m, 0, 0) + j * Vec3(0, 1.0 / m_Grid_n, 0) + Vec3(-0.5, -0.5, 0);
			DUC->drawSphere(pos, Vec3(0.5 / m_Grid_m, 0.5 / m_Grid_n, 0.5 / m_Grid_m));
		}
	}
}


void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawObjects();
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
