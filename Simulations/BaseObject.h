#ifndef OBJECT_H
#define OBJECT_H

#include "Simulator.h"

class BaseObject
{
	bool _userInteraction;
	bool _fixed;
	double _mass;
	std::string _tag;

	Vec3 _position;
	Vec3 _velocity;
	Vec3 _force;

public:

	BaseObject(Vec3 position, double mass, bool fixed = false, bool userInteraction = true);

	// the behaviour of object can be overridden in derived classes, hence we need to set virtual ~Object()
	virtual ~BaseObject();
	// Getters
	bool fixed() const;
	bool userInteraction() const;
	Vec3 position() const;
	Vec3 velocity() const;
	Vec3 force() const;
	std::string tag() const;
	double mass() const;
	// Setters
	void position(Vec3 position);
	void velocity(Vec3 velocity);
	void addForce(Vec3 f);
	void tag(std::string tag);
	void mass(double mass);
	void fixed(bool fixed);
	
	virtual void addGravity(float factor = 10.0f);
	virtual void resetForce();
	virtual void draw(DrawingUtilitiesClass* DUC) = 0;
};


inline BaseObject::BaseObject(Vec3 position, double mass, bool fixed, bool userInteraction)
	: _position(position),
	_mass(mass),
	_fixed(fixed),
	_userInteraction(userInteraction),
	_velocity(Vec3(0, 0, 0))
{}

inline BaseObject::~BaseObject()
{}

inline bool BaseObject::fixed() const
{
	return _fixed;
}

inline bool BaseObject::userInteraction() const
{
	return _userInteraction;
}

inline Vec3 BaseObject::position() const
{
	return _position;
}

inline Vec3 BaseObject::velocity() const
{
	return _velocity;
}
inline double BaseObject::mass() const
{ 
	return _mass; 
}

inline Vec3 BaseObject::force() const
{
	return _force;
}

inline std::string BaseObject::tag() const
{
	return _tag;
}

inline void BaseObject::position(Vec3 position)
{
	_position = position;
}

inline void BaseObject::velocity(Vec3 velocity)
{
	_velocity = velocity;
}

inline void BaseObject::addForce(Vec3 f)
{
	if(!_fixed)
		_force += f;
}

inline void BaseObject::addGravity(float factor) 
{
	addForce(Vec3(0, -factor * _mass, 0));
}

inline void BaseObject::tag(std::string tag)
{
	_tag = tag;
}

inline void BaseObject::fixed(bool fixed)
{
	_fixed = fixed;
}

inline void BaseObject::mass(double mass)
{
	_mass = mass;
}

inline void BaseObject::resetForce()
{
	_force = Vec3(0, 0, 0);
}

#endif