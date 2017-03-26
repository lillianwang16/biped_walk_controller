#pragma once
#include "Box2D.h"
#include "Cartwheel/World.h"

class MyDestructionListener : public b2DestructionListener {
public:
	void SayGoodbye(b2Fixture* fixture) override;
	void SayGoodbye(b2Joint* joint) override;
};

class MyContactFilter : public b2ContactFilter {
public:
	bool ShouldCollide(b2Fixture* fixtureA, b2Fixture* fixtureB) override;
};

class MyContactListener : public b2ContactListener {
public:
	MyContactListener(CartwheelWorld* cw);
	virtual ~MyContactListener() {}
	void BeginContact(b2Contact* contact) override;
	void EndContact(b2Contact* contact) override;
	void PreSolve(b2Contact* contact, const b2Manifold* oldManifold) override;
	void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) override;
	CartwheelWorld* cw_;
	MyContactListener() = delete;
};

class QueryCallback : public b2QueryCallback {
public:
	b2Vec2 point_;
	b2Fixture *fixture_;
	QueryCallback(const b2Vec2& point);
	bool ReportFixture(b2Fixture *fixture) override;
};
