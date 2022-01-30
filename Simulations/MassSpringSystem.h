#pragma once

#include "BaseObject.h"
#include "Simulator.h"
#include "DrawingUtilitiesClass.h"
#include "util/vectorbase.h"
#include "RigidBody.h"

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

	void init(shared_ptr<RigidBody> m1, shared_ptr<RigidBody> m2);

	void applyForceToMasses();

	void draw(DrawingUtilitiesClass* DUC);

private:
	shared_ptr<RigidBody> _m1;
	shared_ptr<RigidBody> _m2;

	Vec3 _direction; // m1 to m2

	float _stiffness;
	float _restLength;
	float _currLength;
};

