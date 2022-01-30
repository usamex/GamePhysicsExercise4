#ifndef EXERCISE4_h
#define EXERCISE4_h
#include "Simulator.h"
#include "MassSpringSystem.h"

class Exercise4 :public Simulator {
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

	void addSpring(int masspoint1, int masspoint2, float initialLength, float stiffness); 
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	
	void applyExternalForce(Vec3 force);

	void testUpdate(float timeStep, int integrator);
	void testEuler(float timeStep);
	void testMidpoint(float timeStep);

private:
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void checkCollision();
	void _update(float timeStep, int integrator, bool addGravity = false);
	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	float collisionBounciness_;
	float stiffness_;
	bool gravity_;
	int direction_;
	int integrationMethod_;

	vector<shared_ptr<RigidBody>> t_rigidBodies_;
	vector<Spring> t_springs_;

};
#endif