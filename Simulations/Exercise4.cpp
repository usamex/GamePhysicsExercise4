#include "Exercise4.h"
#include <sstream>
#include <iostream>
#include <cmath>

void Exercise4::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
}

void Exercise4::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

	gravity_ = false;
}

void Exercise4::externalForcesCalculations(float timeElapsed)
{
	//Point2D mouseDiff;
	//mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	//mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	//if (mouseDiff.x != 0 || mouseDiff.y != 0) {
	//	Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
	//	float scale = 0.01f;
	//	for (int i = 0; i < m_pRigidBodySystem.size(); i++) {
	//		m_pRigidBodySystem[i].applyForce(m_pRigidBodySystem[i].position(), inputView * scale);
	//	}
	//}
	//// also verify gravity
	//if (gravity_) {
	//	for (int i = 0; i < m_pRigidBodySystem.size(); i++) {
	//		m_pRigidBodySystem[i].applyForce(m_pRigidBodySystem[i].position(), m_pRigidBodySystem[i].mass() * _gravity);
	//	}
	//}
}