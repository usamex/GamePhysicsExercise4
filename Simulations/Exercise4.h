#ifndef EXERCISE4_h
#define EXERCISE4_h
#include "Simulator.h"
#include "MassSpringSystem.h"
class Exercise4:public Simulator{
public:
	// Construtors
	Exercise4();
	// UI Functions
	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}
private:
	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	int m_iIntegrator;

	vector<shared_ptr<MassPoint>> t_massPoints_;
	vector<Spring> t_springs_;

	int integrationMethod_;
	bool gravity_;

};
#endif