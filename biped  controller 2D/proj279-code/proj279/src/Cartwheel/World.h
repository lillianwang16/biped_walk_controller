#pragma once
#include "Box2D.h"
#include <vector>

#include "Cartwheel/Character.h"
#include "Cartwheel/Controller.h"

using namespace std;

class CartwheelWorld {
public:
	CartwheelWorld(b2World *world) : world_(world) {}
	void addArticulatedFigure(Character* articulatedFigure) {
		chas_.push_back(articulatedFigure);
		articulatedFigure->addJointsToList(joints_);
	};
	void addController(Controller* controller) {
		controller_list_.push_back(controller);
	}
	void addContact(void* obj1, void *obj2, b2Contact *contact,
	 const b2ContactImpulse *impulse, double dt) {
		ArticulatedRigidBody *rb1 = obj1 == nullptr ? nullptr : (ArticulatedRigidBody*)obj1;
		ArticulatedRigidBody *rb2 = obj2 == nullptr ? nullptr : (ArticulatedRigidBody*)obj2;

		if (rb1 == nullptr && rb2 == nullptr) return;
		
		b2WorldManifold world_manifold;
		contact->GetWorldManifold(&world_manifold);
		for (int i = 0; i < impulse->count; ++i) {
			ContactPoint *contact_point;
			if (rb1 != nullptr) {
				contact_point = new ContactPoint();
				contact_point->cp = world_manifold.points[i];
				contact_point->rb1 = rb1;
				contact_point->rb2 = rb2;
				contact_point->f.x = -(world_manifold.normal.x * impulse->normalImpulses[i]) / dt;
				contact_point->f.y = -(world_manifold.normal.y * impulse->normalImpulses[i]) / dt;
				rb1->afParent_->addContact(contact_point);
			}			
			if (rb2 != nullptr) {
				contact_point = new ContactPoint();
				contact_point->cp = world_manifold.points[i];
				contact_point->rb1 = rb1;
				contact_point->rb2 = rb2;
				contact_point->f.x = -(world_manifold.normal.x * impulse->normalImpulses[i]) / dt;
				contact_point->f.y = -(world_manifold.normal.y * impulse->normalImpulses[i]) / dt;
				rb2->afParent_->addContact(contact_point);
			}
		}
	}

	void performPreTasks(double dt) {
		for (auto& controller : controller_list_)
			controller->performPreTasks(dt, controller->character_->contacts_);
	}
	void advanceInTime(double deltaT) {
		if (deltaT <= 0) return;
		for (auto& j : joints_) {
			double t = j->torque_;
			//we will apply to the parent a positive torque, and to the child a negative torque
			j->GetParent()->body_->ApplyTorque(t, true);
			j->GetChild()->body_->ApplyTorque(-t, true);
		}
		for (auto& cha : chas_)
			cha->clearContacts();
	}

	void performPostTasks(double dt) {
		for (auto& controller : controller_list_)
			controller->performPostTasks(dt, controller->character_->contacts_);
	}

	static bool shouldCheckForCollisions(ArticulatedRigidBody* rb1, ArticulatedRigidBody* rb2) {

		if (rb1->afParent_ != rb2->afParent_) return true;

		if (rb1->body_part_type_ == rb2->body_part_type_) {
			if (rb1->body_part_type_ == BodyPartType::TorsoAndHead)
				return true;
			else
				return false;
		}
		if ((rb1->body_part_type_ == BodyPartType::TorsoAndHead || rb2->body_part_type_ == BodyPartType::TorsoAndHead) &&
			(rb1->body_part_type_ == BodyPartType::Legs || rb2->body_part_type_ == BodyPartType::Legs))
			return true;
		return false;
	}

	b2World *world_;
	vector<Character*> chas_;
	vector<Joint*> joints_;
	vector<Controller*> controller_list_;
};

inline double Min(double a, double b) {
	return a < b ? a : b;
}