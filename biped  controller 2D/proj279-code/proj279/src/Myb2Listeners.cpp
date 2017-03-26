#include "Myb2Listeners.h"
#include "ofApp.h"
#include <iostream>
using namespace std;

void MyDestructionListener::SayGoodbye(b2Fixture* fixture) {
	cout << "Error: Not Done" << endl;
}
void MyDestructionListener::SayGoodbye(b2Joint* joint) {}

bool MyContactFilter::ShouldCollide(b2Fixture* fixtureA, b2Fixture* fixtureB) {
	void* udA = fixtureA->GetBody()->GetUserData();
	void* udB = fixtureB->GetBody()->GetUserData();
	if (udA != NULL && udB != NULL) {
		return CartwheelWorld::shouldCheckForCollisions((ArticulatedRigidBody*)udA,
			(ArticulatedRigidBody*)udB);
	}
	return true;
}

MyContactListener::MyContactListener(CartwheelWorld* cw) : b2ContactListener(), cw_(cw) {}
void MyContactListener::BeginContact(b2Contact* contact) {}
void MyContactListener::EndContact(b2Contact* contact) {}
void MyContactListener::PreSolve(b2Contact* contact, const b2Manifold* oldManifold) {}
void MyContactListener::PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) {
	cw_->addContact(contact->GetFixtureA()->GetBody()->GetUserData(),
		contact->GetFixtureB()->GetBody()->GetUserData(), contact, impulse, 0.0005);

}

QueryCallback::QueryCallback(const b2Vec2& point) : point_(point), fixture_(nullptr) {}
bool QueryCallback::ReportFixture(b2Fixture *fixture) {
	if (fixture->GetBody()->GetType() == b2_dynamicBody) {
		if (fixture->TestPoint(point_)) {
			fixture_ = fixture;
			// We are done, terminate the query.
			return false;
		}
	}
	return true;
}
