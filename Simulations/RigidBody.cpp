#include "RigidBody.h"

void RigidBody::initInertiaInv()
{
	float xx = (1.0 / 12) * _mass * (_size.y * _size.y + _size.z * _size.z);
	float yy = (1.0 / 12) * _mass * (_size.x * _size.x + _size.z * _size.z);
	float zz = (1.0 / 12) * _mass * (_size.x * _size.x + _size.y * _size.y);
	_inertiaTensorInv = Mat4(xx, 0, 0, 0,
		0, yy, 0, 0,
		0, 0, zz, 0,
		0, 0, 0, 1).inverse();
}

void RigidBody::addGravity(float factor) {
	applyForce(position(), Vec3(0, -10 * _mass, 0));
}

void RigidBody::resetForce()
{
	BaseObject::resetForce();
	_torque = Vec3(0, 0, 0);
}

void RigidBody::applyForce(Vec3 pos, Vec3 force)
{
	addForce(force);
	_torque += cross(pos - position(), force); // torque is wrt to relative position pos - position()
}

Mat4 RigidBody::scaleMatrix() const
{
	Mat4 scaleMatrix;
	scaleMatrix.initScaling(_size.x, _size.y, _size.z);
	return scaleMatrix;
}

Mat4 RigidBody::rotationMatrix() const
{
	return _orientation.getRotMat();
}

Mat4 RigidBody::translationMatrix() const
{
	Mat4 transMat;
	transMat.initTranslation(position().x, position().y, position().z);
	return transMat;
}

Mat4 RigidBody::obj2World() const
{
	return scaleMatrix() * rotationMatrix() * translationMatrix();
}

void RigidBody::draw(DrawingUtilitiesClass* DUC)
{
	DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));
	DUC->drawRigidBody(obj2World());
}
Mat4 RigidBody::inertiaTensorInv() const
{
	Mat4 rotMatT = rotationMatrix();
	rotMatT.transpose();
	return rotationMatrix() * _inertiaTensorInv * rotMatT;
}

double RigidBody::distTo(RigidBody& other)
{
	return std::sqrt(position().squaredDistanceTo(other.position()));
}

Vec3 RigidBody::directionTo(RigidBody& other)
{
	return getNormalized(position() - other.position());
}

void RigidBody::eulerUpdate(float timeStep) {
	_oldPosition = position();
	_oldVelocity = velocity();

	position(position() + velocity() * timeStep);
	velocity(velocity() + force() / mass() * timeStep); // acceleration * timeStep
}

void RigidBody::fullStepUpdate(float timeStep) { // for midpoint
	position(_oldPosition + velocity() * timeStep);
	velocity(_oldVelocity + force() / mass() * timeStep); // acceleration * timeStep
}

void RigidBody::update(float timeStep, int integrator, bool _addGravity) {
	if (fixed()) return;

	if (_addGravity) addGravity();

	switch (integrator)
	{
	case EULER: eulerUpdate(timeStep); break;
	case MIDPOINT: fullStepUpdate(timeStep); break;
	default: eulerUpdate(timeStep); break;
	}

	// update orientation quaternion
	orientation(orientation() + (timeStep / 2.f) * angularVelQuat() * orientation());
	angularMom(angularMom() + timeStep * torque());
	angularVel(inertiaTensorInv() * angularMom());

	resetForce();
}