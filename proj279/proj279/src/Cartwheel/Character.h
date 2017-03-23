#pragma once
#include "Box2D.h"
#include <iostream>
#include <string>
#include <vector>

using std::string;
using std::vector;

class ArticulatedRigidBody;
class Character;

enum BodyPartType { TorsoAndHead, Arms, Legs };

inline b2Vec2 VectorDelta(const b2Vec2& V1, const b2Vec2& V2) {
	return b2Vec2(V2.x - V1.x, V2.y - V1.y);
}

class Joint {
public:
	Joint(const string& name, b2RevoluteJoint* joint);
	~Joint() {};
	ArticulatedRigidBody* GetChild() {
		return static_cast<ArticulatedRigidBody*>(joint_->GetBodyB()->GetUserData());
	}
	b2Vec2 GetChildJPos() { return joint_->GetLocalAnchorB(); }
	ArticulatedRigidBody* GetParent() {
		return static_cast<ArticulatedRigidBody*>(joint_->GetBodyA()->GetUserData());
	}
	b2Vec2 GetParentJPos() { return joint_->GetLocalAnchorA(); }

	bool GetUseJointLimits() { return joint_->IsLimitEnabled(); };
	void SetUseJointLimits(bool value) { joint_->EnableLimit(value); }

	void setTorque(double t) { torque_ = t; }

	void computeRelativeOrientation(double& aRel);

	void fixJointConstraints();

	b2RevoluteJoint* joint_;
	double torque_;
	string name_;
	int id_;
	
	Joint() = delete;
};

class ArticulatedRigidBody {
public:
	ArticulatedRigidBody(const string& name, BodyPartType type, b2Body* body, double w, double h)
	 : name_(name), body_(body), parentJoint_(nullptr), afParent_(nullptr),
	   body_part_type_(type), width_(w), height_(h) {
		body_->SetUserData(this);
	}
	virtual ~ArticulatedRigidBody() { 
		childJoints_.clear();
	}

	double getMass() { return body_->GetMass(); };

	double getOrientation() { return body_->GetAngle(); }
	void setOrientation(double o) { body_->SetTransform(body_->GetPosition(), o); }

	b2Vec2 getPosition() { return body_->GetPosition(); }
	void setPosition(b2Vec2 p) { body_->SetTransform(p, body_->GetAngle()); }

	b2Vec2 getVelocity() { return body_->GetLinearVelocity(); }
	double getAngularVelocity() { return body_->GetAngularVelocity(); }

	b2Vec2 getWorldCoordinatesVector(const b2Vec2& localVector) {
		return b2Mul(b2Rot(getOrientation()), localVector);
	}
	b2Vec2 getWorldCoordinatesPoint(const b2Vec2& localPoint) {
		return getPosition() + getWorldCoordinatesVector(localPoint);
	}
	b2Vec2 getLocalCoordinatesVector(const b2Vec2& globalVector) {
		return b2Mul(b2Rot(0 - getOrientation()), globalVector);
	}
	b2Vec2 getLocalCoordinatesPoint(const b2Vec2& globalPoint) {
		return getLocalCoordinatesVector(globalPoint) - getLocalCoordinatesVector(getPosition());
	}
	b2Vec2 getAbsoluteVelocityForLocalPoint(const b2Vec2& localPoint) {
		return b2Cross(getAngularVelocity(), getWorldCoordinatesVector(localPoint))
			+ getVelocity();
	}

	string name_;
	b2Body *body_;
	Joint* parentJoint_;
	vector<Joint*> childJoints_;
	Character* afParent_;
	BodyPartType body_part_type_;
	double width_;
    double height_;
	ArticulatedRigidBody() = delete;
};

struct ContactPoint {
	b2Vec2 cp;
	ArticulatedRigidBody *rb1, *rb2;
	b2Vec2 f;
	ContactPoint() : rb1(nullptr), rb2(nullptr) {}
};

class Character {
public:
	Character() : name_(""), fRoot_(nullptr), mass_(0.0) {}
	~Character() {
		for (Joint*& joint : joints_) delete joint;
		for (ArticulatedRigidBody*& arb : ARBs_) delete arb;
		joints_.clear();
		ARBs_.clear();
		clearContacts();
	}
	void setRoot(ArticulatedRigidBody *root) {
		if (fRoot_ == nullptr) {
			root->afParent_ = this;
			fRoot_ = root;
		}
		else {
			std::cout << "ERROR: fRoot already exists.";
		}
	}
	void computeMass() {
		double total_mass = fRoot_->getMass();
		for (int i = 0; i < joints_.size(); i++) {
			total_mass += joints_[i]->GetChild()->getMass();
		}
		mass_ = total_mass;
	}
	void setJointsIndexes() { for (int i = 0; i < joints_.size(); ++i) joints_[i]->id_ = i; }
	int GetJointCount() { return joints_.size(); }

	void addArticulatedRigidBody(ArticulatedRigidBody* arb) {
		arb->afParent_ = this;
		ARBs_.push_back(arb);
	}
	void addJoint(Joint* joint) { joints_.push_back(joint); }
	void addContact(ContactPoint* contact) { contacts_.push_back(contact); };
	void clearContacts() { 
		for (auto& c : contacts_) delete c;
		contacts_.clear();
	}

	void addJointsToList(vector<Joint*>& otherJoints) {
		for (int i = 0; i < joints_.size(); ++i)
			otherJoints.push_back(joints_[i]);
	}

	void fixJointConstraints() {
		if (fRoot_ == nullptr) return;
		for (int i = 0; i < fRoot_->childJoints_.size(); ++i)
			fRoot_->childJoints_[i]->fixJointConstraints();
	}
	void completeFigure() {
		setJointsIndexes();
		computeMass();
		fixJointConstraints();
	}

	Joint* getJoint(int i) {
		if (i < 0 || i > joints_.size() - 1) return nullptr;
		else return joints_[i];
	}
	virtual int getJointIndex(Joint* joint) {
		if (joint != nullptr && getJoint(joint->id_) == joint)
			return joint->id_;
	}
	virtual int getJointIndex(const string& name) {
		for (int i = 0; i < joints_.size(); ++i)
			if (joints_[i]->name_ == name) return i;
		return -1;
	}
	Joint* getJointByName(const string& name) {
		int res = getJointIndex(name);
		return res == -1 ? nullptr : joints_[res];
	}

	ArticulatedRigidBody* getARBByName(const string& name) {
		if (fRoot_ && fRoot_->name_ == name) return fRoot_;
		for (int i = 0; i < ARBs_.size(); ++i)
			if (name == ARBs_[i]->name_) return ARBs_[i];
		return nullptr;
	}

	// COM: center of mass
	b2Vec2 getCOM() {
		double total_mass = fRoot_->getMass();
		b2Vec2 COM (fRoot_->getPosition().x * total_mass,
					fRoot_->getPosition().y * total_mass);
		for (int i = 0; i < joints_.size(); i++) {
			double current_mass = joints_[i]->GetChild()->getMass();
			total_mass += current_mass;
			COM.x += joints_[i]->GetChild()->getPosition().x * current_mass;
			COM.y += joints_[i]->GetChild()->getPosition().y * current_mass;
		}
		COM.x /= total_mass;
		COM.y /= total_mass;
		return COM;
	}
	b2Vec2 getCOMVelocity() {
		double total_mass = fRoot_->getMass();
		b2Vec2 COM(fRoot_->getVelocity().x * total_mass,
			fRoot_->getVelocity().y * total_mass);
		for (int i = 0; i < joints_.size(); i++) {
			double current_mass = joints_[i]->GetChild()->getMass();
			total_mass += current_mass;
			COM.x += joints_[i]->GetChild()->getVelocity().x * current_mass;
			COM.y += joints_[i]->GetChild()->getVelocity().y * current_mass;
		}
		COM.x /= total_mass;
		COM.y /= total_mass;
		return COM;
	}

	string name_;
	ArticulatedRigidBody *fRoot_;
	double mass_;
	vector<Joint*> joints_;
	vector<ArticulatedRigidBody*> ARBs_;
	vector<ContactPoint*> contacts_;
};


