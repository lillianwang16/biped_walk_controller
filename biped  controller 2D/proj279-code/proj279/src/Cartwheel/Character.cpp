#include "Cartwheel/Character.h"

Joint::Joint(const string& name, b2RevoluteJoint* joint)
	: joint_(joint), torque_(0.0), name_(name), id_(0) {
	GetParent()->childJoints_.push_back(this);
	GetChild()->parentJoint_ = this;
}

void Joint::computeRelativeOrientation(double& aRel) {
	aRel = GetChild()->getOrientation() - GetParent()->getOrientation();
}

void Joint::fixJointConstraints() {
	b2Vec2 rc, rp;
	if (GetChild() == NULL) return;
	if (GetParent() != NULL) {
		rc = GetChild()
			->getWorldCoordinatesVector(VectorDelta(GetChildJPos(), b2Vec2(0, 0)));
		rp = GetParent()
			->getWorldCoordinatesVector(VectorDelta(GetParentJPos(), b2Vec2(0, 0)));
		GetChild()->setPosition(GetParent()->getPosition() + rc - rp);
	}
	for (int i = 0; i < GetChild()->childJoints_.size(); ++i) {
		GetChild()->childJoints_[i]->fixJointConstraints();
	}
}