#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "Simulator.h"
#include "BaseObject.h"
#include "DrawingUtilitiesClass.h"

#define EULER 0
#define MIDPOINT 1
#define LEAPFROG 2

class RigidBody : public BaseObject
{
private:
	double _mass;
	Vec3 _size;
	Vec3 _torque;
	Vec3 _angularVel;
	Vec3 _angularMom;
	Quat _orientation;
	Mat4 _inertiaTensorInv;
public:
	RigidBody(Vec3 position, Vec3 size, double mass, bool fixed = false, bool userInteraction = false) : BaseObject(position, fixed, userInteraction), _size(size), _mass(mass), _orientation(0, 0, 0, 1) {
		initInertiaInv();
	}
	void initInertiaInv();
	void resetForce() override;
	void applyForce(GamePhysics::Vec3 loc, GamePhysics::Vec3 force);
	void update(float timeStep, int integrator, bool _addGravity);

	Vec3 size() const;
	Vec3 torque() const;
	Vec3 angularVel() const;
	Vec3 angularMom() const;
	Quat angularVelQuat() const;
	Quat orientation() const;
	Mat4 scaleMatrix() const;
	Mat4 rotationMatrix() const;
	Mat4 translationMatrix() const;
	Mat4 obj2World() const;
	Mat4 inertiaTensorInv() const;
	double mass() const;

	void addGravity(float factor = 10.0f) override;

	void size(Vec3 size);
	void torque(Vec3 torque);
	void angularVel(Vec3 angularVel);
	void angularMom(Vec3 angularMom);
	void orientation(Quat orientation);

	double distTo(RigidBody& other);
	Vec3 directionTo(RigidBody& other);

	void draw(DrawingUtilitiesClass* DUC);

private:
	void eulerUpdate(float timeStep);
	void fullStepUpdate(float timeStep);

	Vec3 _oldPosition;
	Vec3 _oldVelocity;
};


inline double RigidBody::mass() const
{
	return _mass;
}

inline Vec3 RigidBody::size() const
{
	return _size;
}

inline Vec3 RigidBody::torque() const
{
	return _torque;
}

inline Vec3 RigidBody::angularVel() const
{
	return _angularVel;
}

inline Vec3 RigidBody::angularMom() const
{
	return _angularMom;
}

inline Quat RigidBody::angularVelQuat() const
{
	return Quat(_angularVel.x, _angularVel.y, _angularVel.z, 0);
}

inline Quat RigidBody::orientation() const
{
	return _orientation;
}

inline void RigidBody::size(Vec3 size)
{
	_size = size;
}

inline void RigidBody::torque(Vec3 torque)
{
	_torque = torque;
}

inline void RigidBody::angularVel(Vec3 angularVel)
{
	_angularVel = angularVel;
}

inline void RigidBody::angularMom(Vec3 angularMom)
{
	_angularMom = angularMom;
}

inline void RigidBody::orientation(Quat orientation)
{
	// this way orientation is always normalized
	_orientation = orientation.unit();
}

#endif