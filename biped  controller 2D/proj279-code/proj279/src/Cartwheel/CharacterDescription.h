#pragma once
#include <iostream>
#include "Box2D.h"
#include "Cartwheel/Character.h"

using std::string;

struct MinMax {
	double min_, max_;
	MinMax(double min, double max) : min_(min), max_(max) {}
	double clamp(double value) {
		if (value < min_) return min_;
		if (value > max_) return max_;
		return value;
	}
};

struct Value {
	Value(double value, double min, double max)
		: minmax_(min, max), value_(value) { }
	
	MinMax minmax_;
	double value_;
	void  SetValue(double value) { value_ = minmax_.clamp(value); }
	double get() { return value_; }	
	Value() = delete;
};

struct Symmetric {
	Symmetric(double value, double min, double max)
		: minmax_(min, max), left_(value), right_(value) {}

	MinMax minmax_;
	double left_, right_;
	
	double getSide(int i) { return i <= 0 ? left_ : right_; }

	void setSide(int i, double value) {
		value = minmax_.clamp(value);
		if (i <= 0) left_ = value;
		if (i >= 0) right_ = value;
	}
	
	void setLeft(double value) { left_ = minmax_.clamp(value); }
	void setRight(double value) { right_ = minmax_.clamp(value); }		
	void forceSymmetric() { left_ = right_; }
};

class CharacterDescription {
public:
	CharacterDescription();

	const double hu_ = 0.2286;
	bool isSymmetric_, indirectVarsAreValid_;

	// 11 native symmetrics
	Symmetric footSizeX_;
	Symmetric footSizeZ_;
	Symmetric ankleRelativePosY_;
	Symmetric lowerLegDiameter_;
	Symmetric upperLegDiameter_;
	Symmetric kneeRelativePosY_;
	Symmetric legRelativeAnchorX_;
	Symmetric upperArmDiameter_;
	Symmetric lowerArmDiameter_;
	Symmetric armSizeY_;
	Symmetric elbowRelativePosY_;

	// 6 indirect symmetrics
	Symmetric legPosX_;
	Symmetric anklePosY_;
	Symmetric kneePosY_;
	Symmetric armPosX_;
	Symmetric elbowPosY_;
	Symmetric wristPosY_;

	// 10 native values
	Value legSizeY_;
	Value pelvisDiameter_;
	Value torsoDiameter_;
	Value trunkSizeY_;
	Value waistRelativePosY_;
	Value chestRelativePosY_;
	Value neckSizeY_;
	Value headSizeX_;
	Value headSizeY_;
	Value headSizeZ_;

	// 6 indirect values
	Value groundPosY_;
	Value hipPosY_;
	Value waistPosY_;
	Value chestPosY_;
	Value shoulderPosY_;
	Value neckPosY_;

	void computeIndirectVars();
	void setSymmetric(bool value);

	ArticulatedRigidBody* createArticulatedBox(b2World *world, const string& name,
		BodyPartType type, double sizex, double sizey, double sizez, double mass,
		double moiScale = 1, double posx = 0, double posy = 0);
	ArticulatedRigidBody* createArticulatedEllipsoid(b2World *world, const string& name,
		BodyPartType type, double radiusx, double radiusy, double radiusz, double mass,
		double moiScale = 1, double posx = 0, double posy = 0);
	ArticulatedRigidBody* createArticulatedCylinder(b2World *world, const string& name,
		BodyPartType type, double height, double radius, double mass,
		double moiScale = 1, double posx = 0, double posy = 0);
	Joint* createJoint(b2World* world, const string& name, b2Vec3 posInParent,
		b2Vec3 posInChild, double minAngle, double maxAngle,
		ArticulatedRigidBody* parent, ArticulatedRigidBody* child);	
	Character* createCharacter(b2World *world, b2Vec2 pos);


	// native
	double GetFootSizeX(int side);
	void SetFootSizeX(int side, double value);
	double GetFootSizeZ(int side);
	void SetFootSizeZ(int side, double value);
	double GetAnkleRelativePosY(int side);
	void SetAnkleRelativePosY(int side, double value);
	double GetLowerLegDiameter(int side);
	void SetLowerLegDiameter(int side, double value);
	double GetUpperLegDiameter(int side);
	void SetUpperLegDiameter(int side, double value);
	double GetLegSizeY();
	void SetLegSizeY(double value);
	double GetKneeRelativePosY(int side);
	void SetKneeRelativePosY(int side, double value);
	double GetLegRelativeAnchorX(int side);
	void SetLegRelativeAnchorX(int side, double value);
	double GetPelvisDiameter();
	void SetPelvisDiameter(double value);
	double GetTorsoDiameter();
	void SetTorsoDiameter(double value);
	double GetTrunkSizeY();
	void SetTrunkSizeY(double value);
	double GetWaistRelativePosY();
	void SetWaistRelativePosY(double value);
	double GetChestRelativePosY();
	void SetChestRelativePosY(double value);
	double GetNeckSizeY();
	void SetNeckSizeY(double value);
	double GetHeadSizeX();
	void SetHeadSizeX(double value);
	double GetHeadSizeY();
	void SetHeadSizeY(double value);
	double GetHeadSizeZ();
	void SetHeadSizeZ(double value);
	double GetUpperArmDiameter(int side);
	void SetUpperArmDiameter(int side, double value);
	double GetLowerArmDiameter(int side);
	void SetLowerArmDiameter(int side, double value);
	double GetArmSizeY(int side);
	void SetArmSizeY(int side, double value);
	double GetElbowRelativePosY(int side);
	void SetElbowRelativePosY(int side, double value);
	double GetGroundPosY();
	void SetGroundPosY(double value);
	//indirect
	double GetLegPosX(int side);
	void ComputeLegPosX(int side);
	void SetLegPosX(int side, double value);
	double GetAnklePosY(int side);
	void ComputeAnklePosY(int side);
	void SetAnklePosY(int side, double value);
	double GetKneePosY(int side);
	void ComputeKneePosY(int side);
	void SetKneePosY(int side, double value);
	double GetHipPosY();
	void ComputeHipPosY();
	void SetHipPosY(double value);
	double GetWaistPosY();
	void ComputeWaistPosY();
	void SetWaistPosY(double value);
	double GetChestPosY();
	void ComputeChestPosY();
	void SetChestPosY(double value);
	double GetShoulderPosY();
	void ComputeShoulderPosY();
	void SetShoulderPosY(double value);
	double GetNeckPosY();
	void ComputeNeckPosY();
	void SetNeckPosY(double value);
	double GetArmPosX(int side);
	void ComputeArmPosX(int side);
	double GetElbowPosY(int side);
	void ComputeElbowPosY(int side);
	void SetElbowPosY(int side, double value);
	double GetWristPosY(int side);
	void ComputeWristPosY(int side);
	void SetWristPosY(int side, double value);
};