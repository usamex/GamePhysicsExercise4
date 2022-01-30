#pragma once

#include "BaseObject.h"
#include "Simulator.h"
#include "DrawingUtilitiesClass.h"
#include "util/vectorbase.h"

#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2

class MassPoint: public BaseObject
{
public:
	MassPoint() : BaseObject(Vec3(0, 0, 0), 0) {}
	MassPoint(double mass) : BaseObject(Vec3(0,0,0), mass) {}
	double distTo(MassPoint& other);
	Vec3 directionTo(MassPoint& other);
	void update(float timeStep, int integrator, bool _addGravity = false);
	void draw(DrawingUtilitiesClass* DUC);
private:
	void eulerUpdate(float timeStep);
	void fullStepUpdate(float timeStep);
	Vec3 _oldPosition;
	Vec3 _oldVelocity;
};


class Spring
{
public:
	Spring(float stiffness, float restLength) :
		_stiffness(stiffness),
		_restLength(restLength),
		_currLength(0) {}

	void stiffness(float stiffness) { _stiffness = stiffness; }
	void restLength(float restLength) { _restLength = restLength; }

	double getLength() const;

	void init(shared_ptr<MassPoint> m1, shared_ptr<MassPoint> m2);

	void applyForceToMasses();

	void draw(DrawingUtilitiesClass* DUC);

private:
	shared_ptr<MassPoint> _m1;
	shared_ptr<MassPoint> _m2;

	Vec3 _direction; // m1 to m2

	float _stiffness;
	float _restLength;
	float _currLength;
};

