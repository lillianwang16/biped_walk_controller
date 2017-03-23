#pragma once
#include "Box2D.h"
#include "Cartwheel/Controller.h"

class WorldOracleRayCastCallback : public b2RayCastCallback {
public:
	WorldOracleRayCastCallback() {}
	bool hit_;
	b2Vec2 point_;
	float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point,
		const b2Vec2& normal, float32 fraction) override {
		void * userdata = fixture->GetBody()->GetUserData();
		if (userdata != NULL) {
			return -1.0;
		}
		else {
			hit_ = true;
			point_ = point;
			return fraction;
		}
	}
};



class WorldOracle {
public:
	WorldOracle() : world_(nullptr) {}
	void InitializeWorld(b2World *world) { world_ = world; }
	double getWorldHeightAt(b2Vec2 worldLocation) {
		WorldOracleRayCastCallback callback;
		b2Vec2 Up(0.0, 1.0);
		b2Vec2 p1 = worldLocation + Up + Up;
		b2Vec2 p2 = worldLocation - Up - Up;
		world_->RayCast(&callback, p1, p2);
		double res = callback.hit_ ? callback.point_.y : p2.y;
		debug_point_ = b2Vec2(p1.x, res);
		return res;		
	}
	b2World *world_;
	static b2Vec2 debug_point_;
};

