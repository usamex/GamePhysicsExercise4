#include "Exercise4.h"
#include <sstream>
#include <iostream>
#include <cmath>
#include "collisionDetect.h"

Exercise4::Exercise4() :
	gravity_(false),
	integrationMethod_(EULER),
	collisionBounciness_(1),
	stiffness_(1000),
	direction_(1) {
}
const char* Exercise4::getTestCasesStr()
{
	return "Main Demo";
}

void Exercise4::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwType TW_TYPE_INTEGRATION = TwDefineEnumFromString("Integration Method", "EULER, MIDPOINT");

	TwAddVarRW(DUC->g_pTweakBar, "Integration Method", TW_TYPE_INTEGRATION, &integrationMethod_, "");
	TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_BOOLCPP, &gravity_, "");
	TwAddVarRW(DUC->g_pTweakBar, "Spring stiffness", TW_TYPE_FLOAT, &stiffness_, "");
	TwAddVarRW(DUC->g_pTweakBar, "Bounciness", TW_TYPE_FLOAT, &collisionBounciness_, "");
}

void Exercise4::reset()
{
	//m_mouse.x = m_mouse.y = 0;
	//m_trackmouse.x = m_trackmouse.y = 0;
	//m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

	//gravity_ = false;
}

void Exercise4::notifyCaseChanged(int testCase)
{
	//demo123_ = Demo123(); // reset the simulation
	//m_iTestCase = testCase;
	//switch (m_iTestCase)
	//{
	//case -1:
	//	cout << "--Test--\n";
	//case 0:
	//	cout << "--Demo1--\n";
	//	demo123_.test();
	//	break;
	//case 1:
	//	cout << "--Demo2--\n";
	//	break;
	//case 2:
	//	cout << "--Demo3--\n";
	//	break;
	//case 3:
	//	cout << "--Demo4--\n";
	//	demo4_ = Demo4();
	//	break;
	//default: break;
	//}

	t_rigidBodies_.clear();
	t_springs_.clear();

	// plane
	addRigidBody(Vec3(0.0f, -1.052f, 0.0f), Vec3(16.0f, 0.1f, 16.0f), 100.0f);
	t_rigidBodies_.back()->fixed(true);
	
	// anchors
	addRigidBody(Vec3(-3.0f, 3.0f, 0.0f), Vec3(0.1f, 0.1f, 0.1f), 100.0f);
	t_rigidBodies_.back()->fixed(true);

	addRigidBody(Vec3(3.0f, 3.0f, 0.0f), Vec3(0.1f, 0.1f, 0.1f), 100.0f);
	t_rigidBodies_.back()->fixed(true);

	addRigidBody(Vec3(3.0f, 3.0f, 3.0f), Vec3(0.1f, 0.1f, 0.1f), 100.0f);
	t_rigidBodies_.back()->fixed(true);

	addRigidBody(Vec3(-3.0f, 3.0f, 3.0f), Vec3(0.1f, 0.1f, 0.1f), 100.0f);
	t_rigidBodies_.back()->fixed(true);
	
	addRigidBody(Vec3(0.0f, -1.0f, -3.0f), Vec3(0.1f, 0.1f, 0.1f), 100.0f);
	t_rigidBodies_.back()->fixed(true);

	addRigidBody(Vec3(0.0f, 7.0f, 0.0f), Vec3(0.5f, 0.5f, 0.5f), 100.0f);

	addRigidBody(Vec3(0.0f, 5.0f, 5.0f), Vec3(0.5f, 0.5f, 0.5f), 100.0f);

	addRigidBody(Vec3(-3.0f, 0.0f, 0.0f), Vec3(0.5f, 0.5f, 0.5f), 100.0f);
	t_rigidBodies_.back()->velocity(Vec3(-0.40f, -0.0f, 0.0f));

	addRigidBody(Vec3(3.0f, 0.0f, 0.0f), Vec3(0.5f, 0.5f, 0.5f), 100.0f);
	t_rigidBodies_.back()->velocity(Vec3(-0.20f, -0.4f, 0.0f));

	addRigidBody(Vec3(0.0f, 0.0f, 3.0f), Vec3(0.5f, 0.5f, 0.5f), 100.0f);
	t_rigidBodies_.back()->velocity(Vec3(-0.20f, -0.4f, 0.0f));

	addSpring(6, 1, 0.1, stiffness_ * 3);
	addSpring(6, 2, 0.1, stiffness_ * 3);
	addSpring(7, 3, 0.1, stiffness_ * 3);
	addSpring(7, 4, 0.1, stiffness_ * 3);
	addSpring(7, 5, 0.1, stiffness_ * 3);
	addSpring(6, 8, 0.1, stiffness_ * 3);
	addSpring(6, 9, 0.1, stiffness_ * 3);
	addSpring(7, 10, 0.1, stiffness_ * 3);
	addSpring(7, 6, 0.1, stiffness_ * 3);
	addSpring(10, 8, 0.1, stiffness_ * 3);
	addSpring(10, 9, 0.1, stiffness_ * 3);
	//addSpring(4, 2, 0.1, stiffness_ * 3);
	//addSpring(3, 4, 0.1, stiffness_);
	//addSpring(3, 5, 0.1, stiffness_);
	//addSpring(4, 6, 0.1, stiffness_);
	//addSpring(5, 6, 0.1, stiffness_);
	//addSpring(4, 1, 0.1);
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

void Exercise4::simulateTimestep(float timeStep) {
	// update current setup for each frame
	//switch (m_iTestCase)
	//{// handling different cases
	//case -1:
	//	if (m_iIntegrator == EULER) testEuler(timeStep);
	//	else if (m_iIntegrator == MIDPOINT) testMidpoint(timeStep);
	//	break;
	//case 1: demo123_.update(0.005, EULER); break;
	//case 2: demo123_.update(0.005, MIDPOINT); break;
	//case 3: demo4_.update(timeStep, integrationMethod_ ? MIDPOINT : EULER, gravity_); break;
	//default:
	//	break;
	//}

	if (integrationMethod_ == EULER) {
		_update(timeStep, EULER, gravity_);
	}
	else if (integrationMethod_ == MIDPOINT) {
		_update(timeStep / 2, EULER, gravity_);
		_update(timeStep, MIDPOINT, gravity_);
	}
}


void Exercise4::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	for each (auto sp in t_springs_) sp.draw(DUC);
	for each (auto rb in t_rigidBodies_) rb->draw(DUC);
}

void Exercise4::onClick(int x, int y)
{
	//m_trackmouse.x = x;
	//m_trackmouse.y = y;
}

void Exercise4::onMouse(int x, int y)
{
	//m_oldtrackmouse.x = x;
	//m_oldtrackmouse.y = y;
	//m_trackmouse.x = x;
	//m_trackmouse.y = y;
}

int Exercise4::getNumberOfMassPoints() { return t_rigidBodies_.size(); }

int Exercise4::getNumberOfSprings() { return t_springs_.size(); }

void Exercise4::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	t_rigidBodies_.push_back(std::make_shared<RigidBody>(position, size, mass));
}

void Exercise4::addSpring(int rb1, int rb2, float initialLength, float stiffness)
{
	t_springs_.emplace_back(stiffness_, initialLength);
	t_springs_.back().init(t_rigidBodies_.at(rb1), t_rigidBodies_.at(rb2));
}

void Exercise4::_update(float timeStep, int integrator, bool addGravity) {
	for each (auto sp in t_springs_) sp.applyForceToMasses();

	checkCollision();
	for each (auto rb in t_rigidBodies_) rb->update(timeStep, integrator, addGravity);
}


void Exercise4::checkCollision()
{
	// determine collision relative position for each rigidbody
	//Once you find a collision, your collision response function should calculate :
	// vrel, the relative velocity between Aand B at the collision point(in world space).
	// -> its the velocity of point A minus the vel of point B wrt to the relative collision positions
	// If vrel· n > 0, this indicates that the bodies are separating.
	// Otherwise continue to calculate the impulse J, and apply it to both bodies.
	for (int i = 0; i < t_rigidBodies_.size(); ++i) {
		auto bodyA = t_rigidBodies_[i];
		for (int j = i + 1; j < t_rigidBodies_.size(); ++j) {
			auto bodyB = t_rigidBodies_[j];

			if (bodyA->fixed() && bodyB->fixed()) continue;
			
			CollisionInfo info = checkCollisionSAT(bodyA->obj2World(), bodyB->obj2World());

			if (!info.isValid) continue;
			if (bodyA->fixed()) {
				Vec3 v = bodyB->velocity();
				v.y = -v.y * collisionBounciness_;
				bodyB->velocity(v);
			}

			Vec3 colPosA = info.collisionPointWorld - bodyA->position();
			Vec3 colPosB = info.collisionPointWorld - bodyB->position();

			// determine velocities of collision points
			// vw = vi + w x x_i
			Vec3 velA = bodyA->velocity() + cross(bodyA->angularVel(), colPosA);
			Vec3 velB = bodyB->velocity() + cross(bodyB->angularVel(), colPosB);
			Vec3 velRelative = velA - velB;

			// info.normalWorld is the normalized direction of the impulse from B -> A
			// check velRelative dot info.normalWorld < 0, then the bodies are colliding
			// so we need to calculate the impulse J
			// J = -(1+c)vrel*n / (1/massA + 1/massB + (cross(Ia-1 * cross(posA, n), posA) + Ib-1 * cross(posB, n) cross positionB) dot n
			// OBS:  v = Mat.transformVector(v). This will do the calculation of v= v * Mat
			if (dot(velRelative, info.normalWorld) >= 0) continue;

			Vec3 crossA = cross(bodyA->inertiaTensorInv().transformVector(cross(colPosA, info.normalWorld)), colPosA);
			Vec3 crossB = cross(bodyB->inertiaTensorInv().transformVector(cross(colPosB, info.normalWorld)), colPosB);
			float invMassA = 1.0f / bodyA->mass();
			float invMassB = 1.0f / bodyB->mass();
			float J = -(1 + collisionBounciness_) * dot(velRelative, info.normalWorld)
				/ (invMassA + invMassB + dot(crossA + crossB, info.normalWorld));
			
			// apply impulse
			// remember to change sign
			bodyA->velocity(bodyA->velocity() + J * info.normalWorld / bodyA->mass());
			bodyB->velocity(bodyB->velocity() - J * info.normalWorld / bodyB->mass());
			bodyA->angularMom(bodyA->angularMom() + cross(colPosA, J * info.normalWorld));
			bodyB->angularMom(bodyB->angularMom() - cross(colPosB, J * info.normalWorld));
		}
	}

}