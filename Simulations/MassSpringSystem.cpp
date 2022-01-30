#include "MassSpringSystem.h"

void Spring::init(shared_ptr<RigidBody> m1, shared_ptr<RigidBody> m2) {
	_m1 = m1;
	_m2 = m2;
}

double Spring::getLength() const { return _m1->distTo(*_m2); }

void Spring::applyForceToMasses() {
	// update current state
	_currLength = getLength();
	_direction = _m1->directionTo(*_m2);

	// add forces
	Vec3 force = -_stiffness * (_currLength - _restLength) * _direction;
	_m1->applyForce(_m1->position(), force);
	_m2->applyForce(_m2->position(), -force);
}

void Spring::draw(DrawingUtilitiesClass* DUC) {
	DUC->beginLine();
	DUC->drawLine(_m1->position(), Vec3(1, 0, 0), _m2->position(), Vec3(1, 0, 0));
	DUC->endLine();
}
