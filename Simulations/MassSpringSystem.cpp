#include "MassSpringSystem.h"

double MassPoint::distTo(MassPoint& other)
{
	return std::sqrt(position().squaredDistanceTo(other.position()));
}

Vec3 MassPoint::directionTo(MassPoint& other)
{
	return getNormalized(position() - other.position());
}

void MassPoint::eulerUpdate(float timeStep) {
	_oldPosition = position();
	_oldVelocity = velocity();

	position(position() + velocity() * timeStep);
	velocity(velocity() + force() / mass() * timeStep); // acceleration * timeStep
}

void MassPoint::fullStepUpdate(float timeStep) { // for midpoint
	position(_oldPosition + velocity() * timeStep);
	velocity(_oldVelocity + force() / mass() * timeStep); // acceleration * timeStep
}

void MassPoint::update(float timeStep, int integrator, bool _addGravity) {
	if (_addGravity) addGravity();

	switch (integrator)
	{
	case EULER: eulerUpdate(timeStep); break;
	case MIDPOINT: fullStepUpdate(timeStep); break;
	default: eulerUpdate(timeStep); break;
	}
	resetForce();
}

void MassPoint::draw(DrawingUtilitiesClass* DUC)
{
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(0, 1, 0));
	DUC->drawSphere(position(), Vec3(1, 1, 1) * 0.05f);
}


inline void Spring::init(shared_ptr<MassPoint> m1, shared_ptr<MassPoint> m2) {
	_m1 = m1;
	_m2 = m2;
}

inline double Spring::getLength() const { return _m1->distTo(*_m2); }

inline void Spring::applyForceToMasses() {
	// update current state
	_currLength = getLength();
	_direction = _m1->directionTo(*_m2);

	// add forces
	Vec3 force = -_stiffness * (_currLength - _restLength) * _direction;
	_m1->addForce(force);
	_m2->addForce(-force);
}

inline void Spring::draw(DrawingUtilitiesClass* DUC) {
	DUC->beginLine();
	DUC->drawLine(_m1->position(), Vec3(1, 0, 0), _m2->position(), Vec3(1, 0, 0));
	DUC->endLine();
}
