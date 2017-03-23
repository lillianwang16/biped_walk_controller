#include "Cartwheel/CharacterDescription.h"

CharacterDescription::CharacterDescription() :
	isSymmetric_(true),
	indirectVarsAreValid_(false),
	// 11 native sym
	footSizeX_(Symmetric(0.45 * hu_, 0.6*hu_, hu_)),
	footSizeZ_(Symmetric(1 * hu_, 0.1*hu_, 1e9)),
	ankleRelativePosY_(Symmetric(0.05, 0.05, 0.3)),
	lowerLegDiameter_(Symmetric(0.33 * hu_, 0.2*hu_, hu_)),
	upperLegDiameter_(Symmetric(0.4 * hu_, 0.2*hu_, hu_)),
	kneeRelativePosY_(Symmetric(0.52, 0.2, 0.8)),
	legRelativeAnchorX_(Symmetric(0.6, 0.2, 0.8)),
	upperArmDiameter_(Symmetric(0.35 * hu_, 0.2*hu_, hu_)),
	lowerArmDiameter_(Symmetric(0.28 * hu_, 0.2*hu_, hu_)),
	armSizeY_(Symmetric(2.7 * hu_, 1 * hu_, 5 * hu_)),
	elbowRelativePosY_(Symmetric(0.44444444, 0.01, 0.99)),
	// 6 indirect sym
	legPosX_(Symmetric(0, -1e9, 1e9)),
	anklePosY_(Symmetric(0, -1e9, 1e9)),
	kneePosY_(Symmetric(0, -1e9, 1e9)),
	armPosX_(Symmetric(0, -1e9, 1e9)),
	elbowPosY_(Symmetric(0, -1e9, 1e9)),
	wristPosY_(Symmetric(0, -1e9, 1e9)),
	// 10 natvie values
	legSizeY_(Value(4 * hu_, 2 * hu_, 6 * hu_)),
	pelvisDiameter_(Value(1.1 * hu_, 0.1*hu_, 2.5*hu_)),
	torsoDiameter_(Value(1.4 * hu_, hu_, 2.6*hu_)),
	trunkSizeY_(Value(2.66 * hu_, 1.8*hu_, 3.5*hu_)),
	waistRelativePosY_(Value(0.17, 0.1, 0.4)),
	chestRelativePosY_(Value(0.5, 0.2, 0.8)),
	neckSizeY_(Value(0.05 * hu_, 0.01*hu_, 2 * hu_)),
	headSizeX_(Value(0.9 * hu_, 0.1*hu_, 2 * hu_)),
	headSizeY_(Value(1.1 * hu_, 0.1*hu_, 2 * hu_)),
	headSizeZ_(Value(1.0 * hu_, 0.1*hu_, 2 * hu_)),
	//6 indirect values
	groundPosY_(Value(0, -1e9, 1e9)),
	hipPosY_(Value(0, -1e9, 1e9)),
	waistPosY_(Value(0, -1e9, 1e9)),
	chestPosY_(Value(0, -1e9, 1e9)),
	shoulderPosY_(Value(0, -1e9, 1e9)),
	neckPosY_(Value(0, -1e9, 1e9)) {}

void CharacterDescription::computeIndirectVars() {
	if (indirectVarsAreValid_) return;
	SetGroundPosY(0.0);
	ComputeHipPosY();
	ComputeShoulderPosY();
	ComputeWaistPosY();
	ComputeChestPosY();
	ComputeNeckPosY();
	for (int i = -1; i <= 1; i+=2) {
		ComputeLegPosX(i);
		ComputeLegPosX(i);
		ComputeAnklePosY(i);
		ComputeKneePosY(i);
		ComputeArmPosX(i);
		ComputeWristPosY(i);
		ComputeElbowPosY(i);
	}
}

void CharacterDescription::setSymmetric(bool value) {
	if (isSymmetric_ == value) return;
	isSymmetric_ = value;
	indirectVarsAreValid_ = false;
	footSizeX_.forceSymmetric();
	footSizeZ_.forceSymmetric();
	ankleRelativePosY_.forceSymmetric();
	lowerLegDiameter_.forceSymmetric();
	upperLegDiameter_.forceSymmetric();
	kneeRelativePosY_.forceSymmetric();
	legRelativeAnchorX_.forceSymmetric();
	upperArmDiameter_.forceSymmetric();
	lowerArmDiameter_.forceSymmetric();
	armSizeY_.forceSymmetric();
	elbowRelativePosY_.forceSymmetric();
}

Character* CharacterDescription::createCharacter(b2World *world, b2Vec2 pos) {
	const double massScale = 900;
	Joint* joint;
	Character* character = new Character();
	character->name_ = "Instant Character";
	SetGroundPosY(pos.y);

	double pelvisSizeY = GetWaistPosY() - GetHipPosY();
	double pelvisBottomPos = -pelvisSizeY / 2.0 - GetLegSizeY()*0.1;
	double pelvisTopPos = pelvisSizeY / 2.0;
	double pelvisRadius = GetPelvisDiameter() / 2.0;
	double rootPosY = GetHipPosY() + pelvisSizeY / 2.0 + 0.007;
	ArticulatedRigidBody *pelvis =
		createArticulatedBox(world, "pelvis", BodyPartType::TorsoAndHead,
			pelvisRadius*2.0, pelvisSizeY*1.5, pelvisRadius*1.2, -massScale, 3,
			pos.x, rootPosY);
	character->setRoot(pelvis);

	double totalLowerBackSizeY = GetChestPosY() - GetWaistPosY();
	double lowerBackOffsetY = 0;
	double lowerBackSizeX = GetTorsoDiameter() * 0.7;
	double lowerBackSizeY = totalLowerBackSizeY - lowerBackOffsetY;
	double lowerBackSizeZ = lowerBackSizeX * 0.7;
	ArticulatedRigidBody* lowerback =
		createArticulatedBox(world, "lowerBack", BodyPartType::TorsoAndHead,
			lowerBackSizeX, lowerBackSizeY, lowerBackSizeZ, -massScale);
	character->addArticulatedRigidBody(lowerback);

	joint = createJoint(world, "pelvis_lowerback",
		b2Vec3(0, pelvisSizeY / 2.0, 0),
		b2Vec3(0, -lowerBackSizeY / 2.0 - lowerBackOffsetY, 0),
		-1.6, 1.6, pelvis, lowerback);
	character->addJoint(joint);

	double totalTorsoSizeY = GetShoulderPosY() - GetChestPosY();
	double torsoOffsetY = -0.2 * totalTorsoSizeY;
	double torsoSizeX = GetTorsoDiameter();
	double torsoSizeY = totalTorsoSizeY - torsoOffsetY;
	double torsoSizeZ = torsoSizeX * 0.6;
	ArticulatedRigidBody* torso =
		createArticulatedBox(world, "torso", BodyPartType::TorsoAndHead, torsoSizeX, torsoSizeY, torsoSizeZ, -massScale);
	character->addArticulatedRigidBody(torso);

	joint = createJoint(world, "lowerback_torso", b2Vec3(0, lowerBackSizeY / 2.0, 0),
		b2Vec3(0, -torsoSizeY / 2.0 - torsoOffsetY, 0), -1.6, 1.6, lowerback, torso);
	character->addJoint(joint);

	double headOffsetY = GetNeckSizeY();
	double headSizeX = GetHeadSizeX();
	double headSizeY = GetHeadSizeY();
	double headSizeZ = GetHeadSizeZ();
	ArticulatedRigidBody* head =
		createArticulatedEllipsoid(world, "head", BodyPartType::TorsoAndHead, headSizeX / 2.0, headSizeY / 2.0, headSizeZ / 2.0, -massScale);
	character->addArticulatedRigidBody(head);

	joint = createJoint(world, "torso_head", b2Vec3(0, torsoSizeY / 2.0, 0),
		b2Vec3(0, -headSizeY / 2.0 - headOffsetY, 0), -1.6, 1.6, torso, head);
	character->addJoint(joint);

	double leftUpperArmSizeY = GetShoulderPosY() - GetElbowPosY(1);
	double leftUpperArmDiameter = GetUpperArmDiameter(1);
	ArticulatedRigidBody* lUpperArm =
		createArticulatedCylinder(world, "lUpperArm", BodyPartType::Arms, leftUpperArmSizeY, leftUpperArmDiameter / 2.0, -massScale, 3);
	character->addArticulatedRigidBody(lUpperArm);

	joint = createJoint(world, "lShoulder", b2Vec3(torsoSizeX*0.52, torsoSizeY*0.32, 0),
		b2Vec3(0, leftUpperArmSizeY / 2.0, 0), -1.5, 3.14, torso, lUpperArm);
	character->addJoint(joint);

	double rightUpperArmSizeY = GetShoulderPosY() - GetElbowPosY(-1);
	double rightUpperArmDiameter = GetUpperArmDiameter(-1);
	ArticulatedRigidBody* rUpperArm =
		createArticulatedCylinder(world, "rUpperArm", BodyPartType::Arms, rightUpperArmSizeY, rightUpperArmDiameter / 2.0, -massScale, 3);
	character->addArticulatedRigidBody(rUpperArm);

	joint = createJoint(world, "rShoulder", b2Vec3(-torsoSizeX*0.52, torsoSizeY*0.32, 0),
		b2Vec3(0, rightUpperArmSizeY / 2.0, 0), -1.5, 3.14, torso, rUpperArm);
	character->addJoint(joint);

	double leftLowerArmSizeY = GetElbowPosY(1) - GetWristPosY(1);
	double leftLowerArmDiameter = GetLowerArmDiameter(1);
	ArticulatedRigidBody* lLowerArm =
		createArticulatedCylinder(world, "lLowerArm", BodyPartType::Arms, leftLowerArmSizeY, leftLowerArmDiameter / 2.0, -massScale, 3);
	character->addArticulatedRigidBody(lLowerArm);

	joint = createJoint(world, "lElbow", b2Vec3(0, -leftUpperArmSizeY / 2.0, 0),
		b2Vec3(0, leftLowerArmSizeY / 2.0, 0), 0, 2.7, lUpperArm, lLowerArm);
	character->addJoint(joint);

	double rightLowerArmSizeY = GetElbowPosY(-1) - GetWristPosY(-1);
	double rightLowerArmDiameter = GetLowerArmDiameter(-1);
	ArticulatedRigidBody* rLowerArm =
		createArticulatedCylinder(world, "rLowerArm", BodyPartType::Arms, rightLowerArmSizeY, rightLowerArmDiameter / 2.0, -massScale, 3);
	character->addArticulatedRigidBody(rLowerArm);

	joint = createJoint(world, "rElbow", b2Vec3(0, -rightUpperArmSizeY / 2.0, 0),
		b2Vec3(0, rightLowerArmSizeY / 2.0, 0), 0, 2.7, rUpperArm, rLowerArm);
	character->addJoint(joint);

	double leftUpperLegSizeY = GetHipPosY() - GetKneePosY(1);
	double leftUpperLegDiameter = GetUpperLegDiameter(1);
	ArticulatedRigidBody* lUpperLeg =
		createArticulatedCylinder(world, "lUpperLeg", BodyPartType::Legs, leftUpperLegSizeY, leftUpperLegDiameter / 2.0, -massScale, 4);
	character->addArticulatedRigidBody(lUpperLeg);

	joint = createJoint(world, "lHip", b2Vec3(pelvisRadius*GetLegRelativeAnchorX(1), -pelvisSizeY / 2.0, 0),
		b2Vec3(0, leftUpperLegSizeY / 2.0, 0), -1.3, 1.9, pelvis, lUpperLeg);
	character->addJoint(joint);

	double rightUpperLegSizeY = GetHipPosY() - GetKneePosY(-1);
	double rightUpperLegDiameter = GetUpperLegDiameter(-1);
	ArticulatedRigidBody* rUpperLeg =
		createArticulatedCylinder(world, "rUpperLeg", BodyPartType::Legs, rightUpperLegSizeY, rightUpperLegDiameter / 2.0, -massScale, 4);
	character->addArticulatedRigidBody(rUpperLeg);

	joint = createJoint(world, "rHip", b2Vec3(-pelvisRadius*GetLegRelativeAnchorX(-1), -pelvisSizeY / 2.0, 0),
		b2Vec3(0, rightUpperLegSizeY / 2.0, 0), -1.3, 1.9, pelvis, rUpperLeg);
	character->addJoint(joint);

	double leftLowerLegSizeY = GetKneePosY(1) - GetAnklePosY(1);
	double leftLowerLegDiameter = GetLowerLegDiameter(1);
	ArticulatedRigidBody* lLowerLeg =
		createArticulatedCylinder(world, "lLowerLeg", BodyPartType::Legs, leftLowerLegSizeY, leftLowerLegDiameter / 2.0, -massScale, 4);
	character->addArticulatedRigidBody(lLowerLeg);

	joint = createJoint(world, "lKnee", b2Vec3(0, -leftUpperLegSizeY / 2.0, 0),
		b2Vec3(0, leftLowerLegSizeY / 2.0, 0), -2.5, 0, lUpperLeg, lLowerLeg);
	character->addJoint(joint);

	double rightLowerLegSizeY = GetKneePosY(-1) - GetAnklePosY(-1);
	double rightLowerLegDiameter = GetLowerLegDiameter(-1);
	ArticulatedRigidBody* rLowerLeg =
		createArticulatedCylinder(world, "rLowerLeg", BodyPartType::Legs, rightLowerLegSizeY, rightLowerLegDiameter / 2.0, -massScale, 4);
	character->addArticulatedRigidBody(rLowerLeg);

	joint = createJoint(world, "rKnee", b2Vec3(0, -rightUpperLegSizeY / 2.0, 0),
		b2Vec3(0, rightLowerLegSizeY / 2.0, 0), -2.5, 0, rUpperLeg, rLowerLeg);
	character->addJoint(joint);

	double leftFootSizeX = GetFootSizeX(1);
	double leftFootSizeY = GetAnklePosY(1) - GetGroundPosY();
	double leftFootSizeZ = GetFootSizeZ(1) * 0.75;
	ArticulatedRigidBody* lFoot =
		createArticulatedBox(world, "lFoot", BodyPartType::Legs, leftFootSizeX, leftFootSizeY, leftFootSizeZ, -massScale, 3);
	character->addArticulatedRigidBody(lFoot);

	joint = createJoint(world, "lAnkle", b2Vec3(0, -leftLowerLegSizeY / 2.0, 0),
		b2Vec3(0, leftFootSizeY / 2.0, -leftFootSizeZ*0.33 + leftLowerLegDiameter / 2.0), -0.75, 0.75, lLowerLeg, lFoot);
	character->addJoint(joint);

	double rightFootSizeX = GetFootSizeX(-1);
	double rightFootSizeY = GetAnklePosY(-1) - GetGroundPosY();
	double rightFootSizeZ = GetFootSizeZ(-1) * 0.75;
	ArticulatedRigidBody* rFoot =
		createArticulatedBox(world, "rFoot", BodyPartType::Legs, rightFootSizeX, rightFootSizeY, rightFootSizeZ, -massScale, 3);
	character->addArticulatedRigidBody(rFoot);

	joint = createJoint(world, "rAnkle", b2Vec3(0, -rightLowerLegSizeY / 2.0, 0),
		b2Vec3(0, rightFootSizeY / 2.0, -rightFootSizeZ*0.33 + rightLowerLegDiameter / 2.0), -0.75, 0.75, rLowerLeg, rFoot);
	character->addJoint(joint);

	double leftToesSizeX = leftFootSizeX;
	double leftToesSizeY = leftFootSizeY * 0.66;
	double leftToesSizeZ = GetFootSizeZ(1) - leftFootSizeZ;
	ArticulatedRigidBody* lToes =
		createArticulatedBox(world, "lToes", BodyPartType::Legs, leftToesSizeX, leftToesSizeY, leftToesSizeZ, -massScale, 3);
	character->addArticulatedRigidBody(lToes);

	joint = createJoint(world, "lToeJoint", b2Vec3(0, (leftToesSizeY - leftFootSizeY) / 2.0 + 0.003, leftFootSizeZ / 2.0),
		b2Vec3(0, 0, -leftToesSizeZ / 2.0), -0.1, 0.52, lFoot, lToes);
	character->addJoint(joint);

	double rightToesSizeX = rightFootSizeX;
	double rightToesSizeY = rightFootSizeY * 0.66;
	double rightToesSizeZ = GetFootSizeZ(-1) - rightFootSizeZ;
	ArticulatedRigidBody* rToes =
		createArticulatedBox(world, "rToes", BodyPartType::Legs, rightToesSizeX, rightToesSizeY, rightToesSizeZ, -massScale, 3);
	character->addArticulatedRigidBody(rToes);

	joint = createJoint(world, "rToeJoint",
		b2Vec3(0, (rightToesSizeY - rightFootSizeY) / 2.0 + 0.003, rightFootSizeZ / 2.0),
		b2Vec3(0, 0, -rightToesSizeZ / 2.0), -0.1, 0.52, rFoot, rToes);
	character->addJoint(joint);

	character->completeFigure();

	return character;
}

Joint* CharacterDescription::createJoint(b2World* world, const string& name,
  b2Vec3 posInParent,	b2Vec3 posInChild, double minAngle, double maxAngle,
  ArticulatedRigidBody* parent, ArticulatedRigidBody* child) {
	b2RevoluteJointDef rjd;
	b2RevoluteJoint *rj;
	rjd.Initialize(parent->body_, child->body_, b2Vec2(0,0));
	rjd.localAnchorA = b2Vec2(posInParent.z, posInParent.y);
	rjd.localAnchorB = b2Vec2(posInChild.z, posInChild.y);
	rjd.lowerAngle = minAngle;
	rjd.upperAngle = maxAngle;
	rjd.enableLimit = true;
	rjd.enableMotor = false;
	rj = (b2RevoluteJoint*) world->CreateJoint(&rjd);
	return new Joint(name, rj);
}

ArticulatedRigidBody* CharacterDescription::createArticulatedBox(b2World *world,
  const string& name, BodyPartType type, double sizex, double sizey,
  double sizez, double mass, double moiScale, double posx, double posy) {
	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	b2PolygonShape shape;
	shape.SetAsBox(sizez / 2.0, sizey / 2.0); // 3D to 2D
	b2FixtureDef fd;
	fd.shape = &shape;
	fd.density = 1.0;
	fd.friction = 0.8;
	fd.restitution = 0.35;
	bd.position = b2Vec2(posx, posy);
	b2Body *b = world->CreateBody(&bd);
	b->CreateFixture(&fd);
	if (mass < 0) {
		double volume = sizex * sizey * sizez;
		mass = -mass * volume;
	}
	b2MassData md;
	md.mass = mass;
	md.I = (sizez * sizez + sizey * sizey) * (1.0 / 12.0) * mass * moiScale;
	md.center = b2Vec2(0, 0);
	b->SetMassData(&md);

	return new ArticulatedRigidBody(name, type, b, sizez, sizey);
}

ArticulatedRigidBody* CharacterDescription::createArticulatedEllipsoid(b2World *world,
  const string& name, BodyPartType type, double radiusx, double radiusy, double radiusz,
  double mass, double moiScale, double posx, double posy) {
	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	b2CircleShape shape;
	shape.m_radius = radiusz < radiusy ? radiusz : radiusy;
	b2FixtureDef fd;
	fd.shape = &shape;
	fd.density = 1.0;
	fd.friction = 0.8;
	fd.restitution = 0.35;
	bd.position = b2Vec2(posx, posy);
	b2Body *b = world->CreateBody(&bd);
	b->CreateFixture(&fd);
	if (mass < 0) {
		double volume = 4.0 / 3.0 * 3.141592653 * radiusx * radiusy * radiusz;
		mass = -mass * volume;
	}
	b2MassData md;
	md.mass = mass;
	md.I = mass * (radiusz * radiusz + radiusy * radiusy) / 5.0 * moiScale;
	md.center = b2Vec2(0, 0);
	b->SetMassData(&md);
	return new ArticulatedRigidBody(name, type, b, shape.m_radius, shape.m_radius);
}
ArticulatedRigidBody* CharacterDescription::createArticulatedCylinder(b2World *world,
  const string& name, BodyPartType type, double height, double radius, double mass,
  double moiScale, double posx, double posy) {
	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	b2PolygonShape shape;
	shape.SetAsBox(radius, height / 2); // 3D to 2D
	b2FixtureDef fd;
	fd.shape = &shape;
	fd.density = 1.0;
	fd.friction = 0.8;
	fd.restitution = 0.35;
	bd.position = b2Vec2(posx, posy);
	b2Body *b = world->CreateBody(&bd);
	b->CreateFixture(&fd);
	if (mass < 0) {
		double volume = 3.141592653 * radius * radius * height;
		mass = -mass * volume;
	}
	b2MassData md;
	md.mass = mass;
	md.I = mass * (3 * radius * radius + height * height) / 12.0;
	md.center = b2Vec2(0, 0);
	b->SetMassData(&md);
	return new ArticulatedRigidBody(name, type, b, radius * 2, height);
}

inline bool IsEqual(double a, double b) { return abs(a - b) < 0.00001; }

// native variables

double CharacterDescription::GetFootSizeX(int side) {
	return footSizeX_.getSide(side);
}

void CharacterDescription::SetFootSizeX(int side, double value) {
	if (IsEqual(footSizeX_.getSide(side), value))  return;
	if (isSymmetric_)  side = 0;
	indirectVarsAreValid_ = false;
	footSizeX_.setSide(side, value);;
}

double CharacterDescription::GetFootSizeZ(int side) {
	return footSizeZ_.getSide(side);
}

void CharacterDescription::SetFootSizeZ(int side, double value) {
	if (IsEqual(footSizeZ_.getSide(side), value))  return;
	if (isSymmetric_)  side = 0;
	indirectVarsAreValid_ = false;
	footSizeZ_.setSide(side, value);;
}

double CharacterDescription::GetAnkleRelativePosY(int side) {
	return ankleRelativePosY_.getSide(side);
}

void CharacterDescription::SetAnkleRelativePosY(int side, double value) {
	if (IsEqual(ankleRelativePosY_.getSide(side), value))  return;
	if (isSymmetric_)  side = 0;
	indirectVarsAreValid_ = false;
	ankleRelativePosY_.setSide(side, value);;
}

double CharacterDescription::GetLowerLegDiameter(int side) {
	return lowerLegDiameter_.getSide(side);
}

void CharacterDescription::SetLowerLegDiameter(int side, double value) {
	if (IsEqual(lowerLegDiameter_.getSide(side), value))  return;
	if (isSymmetric_)  side = 0;
	indirectVarsAreValid_ = false;
	lowerLegDiameter_.setSide(side, value);;
}

double CharacterDescription::GetUpperLegDiameter(int side) {
	return upperLegDiameter_.getSide(side);
}

void CharacterDescription::SetUpperLegDiameter(int side, double value) {
	if (IsEqual(upperLegDiameter_.getSide(side), value))  return;
	if (isSymmetric_)  side = 0;
	indirectVarsAreValid_ = false;
	upperLegDiameter_.setSide(side, value);;
}

double CharacterDescription::GetLegSizeY(){
	return legSizeY_.get();
}

void CharacterDescription::SetLegSizeY(double value) {
	if (IsEqual(legSizeY_.value_, value))  return;
	indirectVarsAreValid_ = false;
	legSizeY_.value_ = value;
}

double CharacterDescription::GetKneeRelativePosY(int side) {
	return kneeRelativePosY_.getSide(side);
}

void CharacterDescription::SetKneeRelativePosY(int side, double value) {
	if (IsEqual(kneeRelativePosY_.getSide(side), value))  return;
	if (isSymmetric_)  side = 0;
	indirectVarsAreValid_ = false;
	kneeRelativePosY_.setSide(side, value);;
}

double CharacterDescription::GetLegRelativeAnchorX(int side) {
	return legRelativeAnchorX_.getSide(side);
}

void CharacterDescription::SetLegRelativeAnchorX(int side, double value) {
	if (IsEqual(legRelativeAnchorX_.getSide(side), value))  return;
	if (isSymmetric_)  side = 0;
	indirectVarsAreValid_ = false;
	legRelativeAnchorX_.setSide(side, value);;
}

double CharacterDescription::GetPelvisDiameter() {
	return pelvisDiameter_.get();
}

void CharacterDescription::SetPelvisDiameter(double value) {
	if (IsEqual(pelvisDiameter_.value_, value))  return;
	indirectVarsAreValid_ = false;
	pelvisDiameter_.value_ = value;
}

double CharacterDescription::GetTorsoDiameter(){
	return torsoDiameter_.get();
}

void CharacterDescription::SetTorsoDiameter(double value) {
	if (IsEqual(torsoDiameter_.value_, value))  return;
	indirectVarsAreValid_ = false;
	torsoDiameter_.value_ = value;
}

double CharacterDescription::GetTrunkSizeY() {
	return trunkSizeY_.get();
}

void CharacterDescription::SetTrunkSizeY(double value) {
	if (IsEqual(trunkSizeY_.value_, value))  return;
	indirectVarsAreValid_ = false;
	trunkSizeY_.value_ = value;
}

double CharacterDescription::GetWaistRelativePosY(){
	return waistRelativePosY_.get();
}

void CharacterDescription::SetWaistRelativePosY(double value) {
	if (IsEqual(waistRelativePosY_.value_, value))  return;
	indirectVarsAreValid_ = false;
	waistRelativePosY_.value_ = value;
}

double CharacterDescription::GetChestRelativePosY(){
	return chestRelativePosY_.get();
}

void CharacterDescription::SetChestRelativePosY(double value) {
	if (IsEqual(chestRelativePosY_.value_, value))  return;
	indirectVarsAreValid_ = false;
	chestRelativePosY_.value_ = value;
}

double CharacterDescription::GetNeckSizeY() {
	return neckSizeY_.get();
}

void CharacterDescription::SetNeckSizeY(double value) {
	if (IsEqual(neckSizeY_.value_, value))  return;
	indirectVarsAreValid_ = false;
	neckSizeY_.value_ = value;
}

double CharacterDescription::GetHeadSizeX(){
	return headSizeX_.get();
}

void CharacterDescription::SetHeadSizeX(double value) {
	if (IsEqual(headSizeX_.value_, value))  return;
	indirectVarsAreValid_ = false;
	headSizeX_.value_ = value;
}

double CharacterDescription::GetHeadSizeY() {
	return headSizeY_.get();
}

void CharacterDescription::SetHeadSizeY(double value) {
	if (IsEqual(headSizeY_.value_, value))  return;
	indirectVarsAreValid_ = false;
	headSizeY_.value_ = value;
}

double CharacterDescription::GetHeadSizeZ() {
	return headSizeZ_.get();
}

void CharacterDescription::SetHeadSizeZ(double value) {
	if (IsEqual(headSizeZ_.value_, value))  return;
	indirectVarsAreValid_ = false;
	headSizeZ_.value_ = value;
}

double CharacterDescription::GetUpperArmDiameter(int side) {
	return upperArmDiameter_.getSide(side);
}

void CharacterDescription::SetUpperArmDiameter(int side, double value) {
	if (IsEqual(upperArmDiameter_.getSide(side), value))  return;
	if (isSymmetric_)  side = 0;
	indirectVarsAreValid_ = false;
	upperArmDiameter_.setSide(side, value);;
}

double CharacterDescription::GetLowerArmDiameter(int side) {
	return lowerArmDiameter_.getSide(side);
}

void CharacterDescription::SetLowerArmDiameter(int side, double value) {
	if (IsEqual(lowerArmDiameter_.getSide(side), value))  return;
	if (isSymmetric_)  side = 0;
	indirectVarsAreValid_ = false;
	lowerArmDiameter_.setSide(side, value);;
}

double CharacterDescription::GetArmSizeY(int side) {
	return armSizeY_.getSide(side);
}

void CharacterDescription::SetArmSizeY(int side, double value) {
	if (IsEqual(armSizeY_.getSide(side), value))  return;
	if (isSymmetric_)  side = 0;
	indirectVarsAreValid_ = false;
	armSizeY_.setSide(side, value);;
}

double CharacterDescription::GetElbowRelativePosY(int side) {
	return elbowRelativePosY_.getSide(side);
}

void CharacterDescription::SetElbowRelativePosY(int side, double value) {
	if (IsEqual(elbowRelativePosY_.getSide(side), value))  return;
	if (isSymmetric_)  side = 0;
	indirectVarsAreValid_ = false;
	elbowRelativePosY_.setSide(side, value);;
}

double CharacterDescription::GetGroundPosY(){
	return groundPosY_.get();
}

void CharacterDescription::SetGroundPosY(double value) {
	if (IsEqual(groundPosY_.value_, value))  return;
	indirectVarsAreValid_ = false;
	groundPosY_.value_ = value;
}


// indirect variables

double CharacterDescription::GetLegPosX(int side) {
	if (!indirectVarsAreValid_)  computeIndirectVars();
	return legPosX_.getSide(side);
}

void CharacterDescription::ComputeLegPosX(int side) {
	legPosX_.setSide(side, side*pelvisDiameter_.get() / 2.0*legRelativeAnchorX_.getSide(side));
}

void CharacterDescription::SetLegPosX(int side, double value) {
	SetLegRelativeAnchorX(side, value / pelvisDiameter_.get()*2.0*side);
}

double CharacterDescription::GetAnklePosY(int side) {
	if (!indirectVarsAreValid_)  computeIndirectVars();
	return anklePosY_.getSide(side);
}

void CharacterDescription::ComputeAnklePosY(int side) {
	anklePosY_.setSide(side, groundPosY_.get() + legSizeY_.get() * ankleRelativePosY_.getSide(side));
}

void CharacterDescription::SetAnklePosY(int side, double value) {
	SetAnkleRelativePosY(side, (value - groundPosY_.get()) / legSizeY_.get());
}

double CharacterDescription::GetKneePosY(int side) {
	if (!indirectVarsAreValid_)  computeIndirectVars();
	return kneePosY_.getSide(side);
}

void CharacterDescription::ComputeKneePosY(int side) {
	kneePosY_.setSide(side, anklePosY_.getSide(side) + (hipPosY_.get() - anklePosY_.getSide(side)) * kneeRelativePosY_.getSide(side));
}

void CharacterDescription::SetKneePosY(int side, double value) {
	SetKneeRelativePosY(side, (value - anklePosY_.getSide(side)) / (hipPosY_.get() - anklePosY_.getSide(side)));
}

double CharacterDescription::GetHipPosY() {
	if (!indirectVarsAreValid_)  computeIndirectVars();
	return hipPosY_.get();
}

void CharacterDescription::ComputeHipPosY() {
	hipPosY_.SetValue(groundPosY_.get() + legSizeY_.get());
}

void CharacterDescription::SetHipPosY(double value) {
	SetLegSizeY(value - groundPosY_.get());
}

double CharacterDescription::GetWaistPosY() {
	if (!indirectVarsAreValid_)  computeIndirectVars();
	return waistPosY_.get();
}

void CharacterDescription::ComputeWaistPosY() {
	waistPosY_.SetValue(hipPosY_.get() + trunkSizeY_.get() * waistRelativePosY_.get());
}

void CharacterDescription::SetWaistPosY(double value) {
	SetWaistRelativePosY((value - hipPosY_.get()) / trunkSizeY_.get());
}

double CharacterDescription::GetChestPosY() {
	if (!indirectVarsAreValid_)  computeIndirectVars();
	return chestPosY_.get();
}

void CharacterDescription::ComputeChestPosY() {
	chestPosY_.SetValue(waistPosY_.get() + (shoulderPosY_.get() - waistPosY_.get()) * chestRelativePosY_.get());
}

void CharacterDescription::SetChestPosY(double value) {
	SetChestRelativePosY((value - waistPosY_.get()) / (shoulderPosY_.get() - waistPosY_.get()));
}

double CharacterDescription::GetShoulderPosY() {
	if (!indirectVarsAreValid_)  computeIndirectVars();
	return shoulderPosY_.get();
}

void CharacterDescription::ComputeShoulderPosY() {
	shoulderPosY_.SetValue(hipPosY_.get() + trunkSizeY_.get());
}

void CharacterDescription::SetShoulderPosY(double value) {
	SetTrunkSizeY(value - hipPosY_.get());
}

double CharacterDescription::GetNeckPosY() {
	if (!indirectVarsAreValid_)  computeIndirectVars();
	return neckPosY_.get();
}

void CharacterDescription::ComputeNeckPosY() {
	neckPosY_.SetValue(shoulderPosY_.get() + neckSizeY_.get());
}

void CharacterDescription::SetNeckPosY(double value) {
	SetNeckSizeY(value - shoulderPosY_.get());
}

double CharacterDescription::GetArmPosX(int side) {
	if (!indirectVarsAreValid_)  computeIndirectVars();
	return armPosX_.getSide(side);
}

void CharacterDescription::ComputeArmPosX(int side) {
	armPosX_.setSide(side, side*(torsoDiameter_.get() + upperArmDiameter_.getSide(side)) / 2.0);
}

double CharacterDescription::GetElbowPosY(int side) {
	if (!indirectVarsAreValid_)  computeIndirectVars();
	return elbowPosY_.getSide(side);
}

void CharacterDescription::ComputeElbowPosY(int side) {
	elbowPosY_.setSide(side, shoulderPosY_.get() - armSizeY_.getSide(side)*elbowRelativePosY_.getSide(side));
}

void CharacterDescription::SetElbowPosY(int side, double value) {
	SetElbowRelativePosY(side, (shoulderPosY_.get() - value) / armSizeY_.getSide(side));
}

double CharacterDescription::GetWristPosY(int side) {
	if (!indirectVarsAreValid_)  computeIndirectVars();
	return wristPosY_.getSide(side);
}

void CharacterDescription::ComputeWristPosY(int side) {
	wristPosY_.setSide(side, shoulderPosY_.get() - armSizeY_.getSide(side));
}

void CharacterDescription::SetWristPosY(int side, double value) {
	SetArmSizeY(side, shoulderPosY_.get() - value);
}
