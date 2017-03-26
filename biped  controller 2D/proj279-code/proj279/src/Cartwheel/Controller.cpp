#include "Cartwheel/Controller.h"
#include "Cartwheel/TwoLinkIk.h"

inline double clamp(double value, double min, double max) {
	if (value < min)
		return  min;
	else if (value > max)
		return  max;
	else
		return  value;	
}

inline double Min(double a, double b) { return a < b ? a : b; }

Controller::Controller(Character* ch, const string& name) {
	name_ = name;
	character_ = ch;	
	torques_ = vector<double>(ch->GetJointCount());
	oldTorques_ = vector<double>(ch->GetJointCount());
	noOldTorqueInfo = true;
}

int Controller::GetJointCount() {
	return  torques_.size();
}

void Controller::performPreTasks(double dt, vector<ContactPoint*>& cfs) {
	computeTorques(cfs);
	applyTorques();
}

bool  Controller::performPostTasks(double dt, vector<ContactPoint*>& cfs) {
	return  false;
}

void Controller::applyTorques() {
	const double maxTorqueRateOfChange = 2000;
	int   i;
	double deltaT;
	double tmpTorque; 
	if (noOldTorqueInfo) {
		oldTorques_ = vector<double>(torques_);
		noOldTorqueInfo = false;
	}

	for (i = 0; i < GetJointCount(); ++i) {
		deltaT = torques_[i] - oldTorques_[i];

		deltaT = clamp(deltaT, -maxTorqueRateOfChange, maxTorqueRateOfChange);

		tmpTorque = oldTorques_[i] + deltaT;

		character_->getJoint(i)->setTorque(tmpTorque);
		oldTorques_[i] = tmpTorque;
	}
}

void Controller::resetTorques(){
	noOldTorqueInfo = true;
	torques_ = vector<double>(torques_.size(), 0.0);
}

//------------------------------- ControlParams-------------------------------

ControlParams::ControlParams(Joint* joint) {
	joint_ = joint;
	strength_ = 1;
	controlled_ = false;
	kp_ = 0; kd_ = 0; maxAbsTorque_ = 0; scale_ = 0;
	qRelExternallyComputed_ = false; relToFrame_ = false;
}

string  ControlParams::getJointName() {
	if (joint_ != nullptr)	return(joint_->name_);
	return "root";
}

Joint*  ControlParams::getJoint() {
	return  joint_;
}

//-------------------------------- PoseController ---------------------------------

PoseController::PoseController(Character* ch, const string& name)
	: Controller(ch,name) {
	desiredPose_ = new CharacterState(ch->GetJointCount());
	controlParams_ = vector<ControlParams*>();
	for (int i = 0; i < GetJointCount(); ++i)
		controlParams_.push_back(new ControlParams(ch->getJoint(i)));
}

PoseController::~PoseController() {
	for (auto& cp : controlParams_)
		delete cp;
	controlParams_.clear();
	delete desiredPose_;
}

void PoseController::addControlParams(ControlParams* params){

	int    jIndex; 
	jIndex = character_->getJointIndex(params->getJoint());
	if (jIndex < 0)
		return;
	controlParams_[jIndex] = params;
}

double PoseController::computePDTorque(double aRel, double aRelD, double wRel, double wRelD, ControlParams*  cParams){
	double    torque;
	torque = ((-cParams->kp_ * (aRelD - aRel)) + (-cParams->kd_ * (wRelD - wRel))) * cParams->strength_;
	scaleAndLimitTorque(torque, cParams);
	return  torque;
}

void PoseController::limitTorque(double& torque, ControlParams*  cParams) {
	torque = clamp(torque, -cParams->scale_* cParams->maxAbsTorque_, cParams->scale_* cParams->maxAbsTorque_);
}

void PoseController::scaleAndLimitTorque(double&  torque, ControlParams*  cParams) {
	torque = torque * cParams->scale_;
	limitTorque(torque, cParams);
}

void PoseController::scaleGains(double factor){

	int   i;
	double    kp; double    kd; 
	for (i = 0; i < controlParams_.size(); ++i) {
		if (character_->getJoint(i)->name_.find("ToeJoint") != string::npos) continue;
		kp = controlParams_[i]->kp_ * factor;
		controlParams_[i]->kp_ = kp;
		kd = sqrt(kp) * 2;
		controlParams_[i]->kd_ = kd;
	}
}

void PoseController::computeTorques(vector<ContactPoint*>& cfs){

	CharacterState*   rs;
	int   i;
	ControlParams*   cParams;
	Joint*   joint;
	ArticulatedRigidBody*  parentRB;
	ArticulatedRigidBody*   childRB;
	double parentAworld, frameAworld, frameAngularVelocityInFrame;
	double currentOrientationInFrame, desiredOrientationInFrame;
	double    currentAngularVelocityInFrame;
	double    desiredRelativeAngularVelocityInFrame; double    currentRelativeAngularVelocityInFrame; 
	rs = desiredPose_;

	for (i = 0; i< GetJointCount();++i) {
		cParams = controlParams_[i];
		if (cParams->controlled_) {
			joint = character_->getJoint(i);
			parentRB = joint->GetParent();
			childRB = joint->GetChild();
			parentAworld = -parentRB->getOrientation();

			if (!cParams->relToFrame_) {
				frameAworld = parentAworld;
				frameAngularVelocityInFrame = parentRB->getAngularVelocity();
			}
			else {
				frameAworld = -0;
				frameAngularVelocityInFrame = 0;			}

			currentOrientationInFrame = frameAworld + childRB->getOrientation();
			desiredOrientationInFrame = rs->getJointRelativeOrientation(i);

			currentAngularVelocityInFrame = childRB->getAngularVelocity();
			desiredRelativeAngularVelocityInFrame = rs->getJointRelativeAngVelocity(i);
			currentRelativeAngularVelocityInFrame = currentAngularVelocityInFrame - frameAngularVelocityInFrame;

			torques_[i] = computePDTorque(currentOrientationInFrame, desiredOrientationInFrame, currentRelativeAngularVelocityInFrame,
				desiredRelativeAngularVelocityInFrame, cParams);
		}
		else {
			torques_[i] = 0;
		}
	}
}

//----------------------------------CharacterState------------------------------

CharacterState::CharacterState(int jointCount)
	: position_(0,0), velocity_(0,0),
	orientation_(0), angularVelocity_(0), jointRelatives_(vector<JointRelative>(jointCount, JointRelative())){
}

double  CharacterState::getJointRelativeOrientation(int jIndex) {
	return  jointRelatives_[jIndex].orientation_;
}

void CharacterState::setJointRelativeOrientation(double a, int  jIndex) {
	jointRelatives_[jIndex].orientation_ = a;
}

double  CharacterState::getJointRelativeAngVelocity(int jIndex) {
	return  jointRelatives_[jIndex].angularVelocity_;
}

void CharacterState::setJointRelativeAngVelocity(double w, int  jIndex) {
	jointRelatives_[jIndex].angularVelocity_ = w;
}

//----------------------------------SimBiController-------------------------------------

SimBiController::SimBiController(Character* b, const string& name)
	: PoseController(b, name), stanceFoot_(nullptr), swingFoot_(nullptr), rootControlParams_(nullptr),
	aRootD_(0), stance_(0), stanceHipIndex_(0), swingHipIndex_(0), FSMStateIndex_(0), comVelocity_(0,0),
	comPosition_(0,0), d_(0,0),v_(0,0),doubleStanceCOMError_(0,0),phi_(0), bodyTouchedTheGround_(false),
	startingState_(0),startingStance_(0){

	Joint*    lHip;
	Joint*    rHip;	
	states_ = vector<SimBiConState*>();

	if (b == nullptr) {
		std::cout << __func__ << "Error n nullptr\n";
		return;
	}
	//characters controlled_ by a simbicon controller are assumed to have 2 feet
	lFoot_ = b->getARBByName("lFoot");
	rFoot_ = b->getARBByName("rFoot");

	if ((rFoot_ == nullptr) || (lFoot_ == nullptr)) {
		lFoot_ = b->getARBByName("lFoot_2");
		rFoot_ = b->getARBByName("rFoot_2");
	}

	if ((rFoot_ == nullptr) || (lFoot_ == nullptr)) {
		std::cout << __func__ << "Error foot nullptr\n";
		return;
	}
	//and two hips connected to the root
	lHip = b->getJointByName("lHip");
	rHip = b->getJointByName("rHip");

	lHipIndex_ = b->getJointIndex("lHip");
	rHipIndex_ = b->getJointIndex("rHip");

	if ((rHip == nullptr) || (lHip == nullptr)) {
		std::cout << __func__ << "Error hip nullptr\n";
		return;
	}

	root_ = b->fRoot_;

	if ((lHip->GetParent() != rHip->GetParent()) || (lHip->GetParent() != root_)) {
		std::cout << "The biped""s hips must have a common parent, which should be the root of the figure!" << std::endl;
		return;
	}

	setStance(LEFT_STANCE);
	phi_ = 0;

	setFSMStateTo(-1);

	stanceHipDamping_ = -1;
	stanceHipMaxVelocity_ = 4;
	rootPredictiveTorqueScale_ = 0;

	bodyTouchedTheGround_ = false;

	startingState_ = 0;
	startingStance_ = LEFT_STANCE;
}

SimBiController::~SimBiController(){ }

void SimBiController::scaleGains(double factor) {
	rootControlParams_->kp_ = rootControlParams_->kp_ * factor;
	rootControlParams_->kd_ = rootControlParams_->kd_ * factor;
	// rootControlParams_->kd_ = Sqrt(rootControlParams_->kp_) * 2;
	PoseController::scaleGains(factor);
}

void SimBiController::addControlParams(ControlParams* params) {
	if (params->joint_ == nullptr)
		rootControlParams_ = params;
	else
		PoseController::addControlParams(params);
}

SimBiConState*  SimBiController::getCurrentState() {
	if (FSMStateIndex_ < 0)
		setFSMStateTo(startingState_);
	return  getState(FSMStateIndex_);
}

double* SimBiController::getPhiPtr() {
	return  &phi_;
}

void SimBiController::setFSMStateTo(int index) {
	if (index < 0)
		FSMStateIndex_ = -1;
	else
		if (index >= states_.size())
			FSMStateIndex_ = states_.size() - 1;
		else
			FSMStateIndex_ = index;
}

void SimBiController::transitionToState(int stateIndex) {
	setFSMStateTo(stateIndex);
	setStance(states_[FSMStateIndex_]->getStateStance(stance_));
	phi_ = 0;
}

b2Vec2 SimBiController::getForceOn(ArticulatedRigidBody* rb, vector<ContactPoint*>&  cfs) {

	b2Vec2 fNet;
	int    i; 
	fNet = b2Vec2(0, 0);
	for (i = 0; i < cfs.size();++i){
		if (cfs[i]->rb1 == rb)
			fNet = fNet + cfs[i]->f;
		if (cfs[i]->rb2 == rb)
			fNet = fNet - cfs[i]->f;
	}
	return  fNet;
}

b2Vec2 SimBiController::getForceOnFoot(ArticulatedRigidBody* foot, vector<ContactPoint*>&  cfs){

	b2Vec2   fNet;
	fNet = getForceOn(foot, cfs);

	//we will also look at all children of the foot that is passed in (to take care of toes).
	for (int i = 0; i < foot->childJoints_.size(); ++i)
		fNet = fNet + getForceOn(foot->childJoints_[i]->GetChild(), cfs);

	return  fNet;
}

bool SimBiController::haveRelationBetween(ArticulatedRigidBody* rb, ArticulatedRigidBody* whichBody){

	int    j; 
	//check against the feet
	if (rb == whichBody)
		return(true);
	for (j = 0; j< whichBody->childJoints_.size(); ++j)
		if (whichBody->childJoints_[j]->GetChild() == rb)
			return true;
	return(false);
}

bool  SimBiController::isFoot(ArticulatedRigidBody* rb) {
	return(haveRelationBetween(rb, lFoot_) || haveRelationBetween(rb, rFoot_));
}

bool  SimBiController::isStanceFoot(ArticulatedRigidBody* rb) {
	return(haveRelationBetween(rb, stanceFoot_));
}

bool  SimBiController::isSwingFoot(ArticulatedRigidBody* rb) {
	return(haveRelationBetween(rb, swingFoot_));
}

double SimBiController::getStanceFootWeightRatio(vector<ContactPoint*>& cfs) {

	b2Vec2   stanceFoot_Force; b2Vec2   swingFoot_Force;
	double    totalYForce; 
	stanceFoot_Force = getForceOnFoot(stanceFoot_, cfs);
	swingFoot_Force = getForceOnFoot(swingFoot_, cfs);
	totalYForce = b2Dot(stanceFoot_Force + swingFoot_Force, Up);

	if (abs(totalYForce) < 1e-6)
		return(-1);
	else
		return(b2Dot(stanceFoot_Force, Up) / totalYForce);
}

void SimBiController::computeHipTorques(double aRootD_, double stanceHipToSwingHipRatio, double ffRootTorque_){

	double rootTorque, swingHip_Torque, aRootD_W, rootStrength, rootMakeupTorque;
	double   stanceHipTorque; double   wRel; double   wRelLen;
	int    i;
	if (stanceHipToSwingHipRatio < 0)
		rootControlParams_->strength_ = 0;

	aRootD_W = 0 + aRootD_;

	rootStrength = clamp(rootControlParams_->strength_, 0, 1);

	rootControlParams_->strength_ = 1;

	//so this is the net torque that the root wants to see, in world coordinates
	rootTorque = computePDTorque(root_->getOrientation(), aRootD_W, root_->getAngularVelocity(), 0, rootControlParams_);

	rootTorque = rootTorque + ffRootTorque_;

	swingHip_Torque = torques_[swingHipIndex_];

	//we need to compute the total torque that needs to be applied at the hips so that the final net torque on the root is rootTorque
	rootMakeupTorque = 0;
	for (i = 0; i < GetJointCount(); ++i)
		if (character_->getJoint(i)->GetParent() == root_)
			rootMakeupTorque = rootMakeupTorque - torques_[i];
	rootMakeupTorque = rootMakeupTorque - rootTorque;

	//assume the stance foot is in contact...
	stanceHipTorque = torques_[stanceHipIndex_];

	//now, based on the ratio of the forces the feet exert on the ground, && the ratio of the forces between the two feet, we will compute the forces that need to be applied
	//to the two hips, to make up the root makeup torque - which means the final torque the root sees is rootTorque!
	stanceHipTorque = stanceHipTorque + rootMakeupTorque * stanceHipToSwingHipRatio * rootStrength;
	swingHip_Torque = swingHip_Torque + rootMakeupTorque * (1 - stanceHipToSwingHipRatio) * rootStrength;


	if (stanceHipDamping_ > 0) {
		wRel = root_->getAngularVelocity() - character_->joints_[stanceHipIndex_]->GetChild()->getAngularVelocity();
		wRelLen = abs(wRel);
		if (wRelLen > stanceHipMaxVelocity_) wRel = wRel * (stanceHipMaxVelocity_ / wRelLen);
		stanceHipTorque = stanceHipTorque - wRel * (stanceHipDamping_ * wRelLen);
	}

	limitTorque(stanceHipTorque, controlParams_[stanceHipIndex_]);

	limitTorque(swingHip_Torque, controlParams_[swingHipIndex_]);

	//and)ne...
	torques_[stanceHipIndex_] = stanceHipTorque;
	torques_[swingHipIndex_] = swingHip_Torque;
}

void SimBiController::resolveJoints(SimBiConState *state) {

	int   i;
	Trajectory   *jt;
	string    tmpName; 
	for (int i = 0; i < state->sTraj_.size(); ++i) {
		jt = state->sTraj_[i];
		//deal with the "root" special case
		if (jt->jName_ == "root") {
			jt->leftStanceIndex_ = -1;
			jt->rightStanceIndex_ = -1;
			continue;
		}
		//deal with the SWING_XXX" case
		if (jt->jName_.find("SWING_") != string::npos) {
			tmpName = " " + jt->jName_.substr(6);
			tmpName[0] = 'r';			
			jt->leftStanceIndex_ = character_->getJointIndex(tmpName);
			if (jt->leftStanceIndex_ < 0) {
				std::cout << "Cannot find joint " << tmpName << std::endl;
				return;
			}
			tmpName[0] = 'l';
			jt->rightStanceIndex_ = character_->getJointIndex(tmpName);
			if (jt->rightStanceIndex_ < 0) {
				std::cout << "Cannot find joint " << tmpName << std::endl;
				return;
			}
			continue;
		}
		//deal with the STANCE_XXX" case
		if (jt->jName_.find("STANCE_") != string::npos) {
			tmpName = " " + jt->jName_.substr(7);
			tmpName[0] = 'l';
			jt->leftStanceIndex_ = character_->getJointIndex(tmpName);
			if (jt->leftStanceIndex_ < 0) {
				std::cout << "Cannot find joint " << tmpName << std::endl;
				return;
			}
			tmpName[0] = 'r';
			jt->rightStanceIndex_ = character_->getJointIndex(tmpName);
			if (jt->rightStanceIndex_ < 0) {
				std::cout << "Cannot find joint " << tmpName << std::endl;
				return;
			}
			continue;
		}
		//if (we get here, it means it is just the name...
		jt->leftStanceIndex_ = character_->getJointIndex(jt->jName_);
		if (jt->leftStanceIndex_ < 0) {
			std::cout << "Cannot find joint " << jt->jName_ << std::endl;
			return;
		}
		jt->rightStanceIndex_ = jt->leftStanceIndex_;
	}
}

void SimBiController::setStance(int newStance) {
	stance_ = newStance;
	if (stance_ == LEFT_STANCE) {
		stanceFoot_ = lFoot_;
		swingFoot_ = rFoot_;
		stanceHipIndex_ = lHipIndex_;
		swingHipIndex_ = rHipIndex_;
	} else {
		stanceFoot_ = rFoot_;
		swingFoot_ = lFoot_;
		stanceHipIndex_ = rHipIndex_;
		swingHipIndex_ = lHipIndex_;
	}
}

SimBiConState* SimBiController::getState(int idx) {
	if (idx >= states_.size()) return(nullptr);
	return(states_[idx]);
}

int  SimBiController::getStateCount() {
	return(states_.size());
}

void SimBiController::addState(SimBiConState* state_disown) {
	states_.push_back(state_disown);
	resolveJoints(state_disown);
}

void SimBiController::clearStates(){
	states_.clear();
}

void SimBiController::computeTorques(vector<ContactPoint*>& cfs) {
	if (FSMStateIndex_ < 0)
		setFSMStateTo(startingState_);

	if (FSMStateIndex_ >= states_.size()) {
		// WriteLn("Warning: no FSM state was selected in the controller!");
		return;
	}

	evaluateJointTargets();
	computePDTorques(cfs);
	//and now separetely compute the torques_ for (the hips - together with the feedback term, this is what defines simbicon
	computeHipTorques(aRootD_, getStanceFootWeightRatio(cfs), 0);
	// bl}OutTorques();
}

void SimBiController::evaluateJointTargets() {

	CharacterState*   poseRS;
	int   i;
	double   phiToUse;
	SimBiConState* curState;
	double   newOrientation;
	int   jIndex;
	b2Vec2    d0; b2Vec2    v0;
	poseRS = desiredPose_;
	updateDAndV();
	//always start from a neutral desired pose, && build from there...
	for (i = 0; i < GetJointCount(); i++) {
		if (!controlParams_[i]->qRelExternallyComputed_) {
			poseRS->setJointRelativeOrientation(0, i);
			poseRS->setJointRelativeAngVelocity(0, i);
		}
		controlParams_[i]->controlled_ = true;
		controlParams_[i]->relToFrame_ = false;
	}
	aRootD_ = 0;
	phiToUse = Min(phi_, 1);
	curState = states_[FSMStateIndex_];

	for (i = 0; i < curState->getTrajectoryCount(); ++i) {
		jIndex = curState->sTraj_[i]->getJointIndex(stance_);
	if ((jIndex > -1) && controlParams_[jIndex]->qRelExternallyComputed_)
		continue;

	computeD0(phiToUse, d0);
	computeV0(phiToUse, v0);
	newOrientation = curState->sTraj_[i]
		->evaluateTrajectory(this, character_->getJoint(jIndex), stance_, phiToUse, d_ - d0, v_ - v0);

	if (jIndex == -1) {
		aRootD_ = newOrientation;
	rootControlParams_->strength_ = curState->sTraj_[i]->evaluateStrength(phiToUse);
		}
	else
		{
		if (curState->sTraj_[i]->relToCharFrame_ || (jIndex = swingHipIndex_))
			controlParams_[jIndex]->relToFrame_ = true;
			poseRS->setJointRelativeOrientation(newOrientation, jIndex);
			controlParams_[jIndex]->strength_ = curState->sTraj_[i]->evaluateStrength(phiToUse);
		}
	}
}

void SimBiController::computePDTorques(vector<ContactPoint*>& cfs) {
	PoseController::computeTorques(cfs);
}

bool SimBiController::performPostTasks(double dt, vector<ContactPoint*>&  cfs){

	bool    transition; 
	PoseController::performPostTasks(dt, cfs);
	transition = advanceInTime(dt, cfs) != -1;
	updateDAndV();
	return  transition;
}

int SimBiController::advanceInTime(double dt, vector<ContactPoint*>&  cfs) {
	int    i; int    newStateIndex;
	if (FSMStateIndex_ < 0)
		setFSMStateTo(startingState_);

	if (dt <= 0)
		return(-1);

	if (FSMStateIndex_ >= states_.size()) {
		// WriteLn("Warning: no FSM state was selected in the controller!");
		return(-1);
	}

	bodyTouchedTheGround_ = false;
	//see if (anything else other than the feet touch the ground...
	for (i = 0; i < cfs.size(); ++i) {
		if (isFoot(cfs[i]->rb1) || isFoot(cfs[i]->rb2))
			continue;
		bodyTouchedTheGround_ = true;
		break;
	}

	//advance the phase of the controller
	phi_ = phi_ + dt / states_[FSMStateIndex_]->stateTime_;

	//see if (we have to transition to the next state in the FSM, and) it if (so...
	if (states_[FSMStateIndex_]->needTransition(phi_, abs(b2Dot(getForceOnFoot(swingFoot_, cfs), Up)), abs(b2Dot(getForceOnFoot(stanceFoot_, cfs), Up)))) {
		newStateIndex = states_[FSMStateIndex_]->nextStateIndex_;
		transitionToState(newStateIndex);
		return(newStateIndex);
	}

	//if (we didn"t transition to a new state->..
	return(-1);
}

b2Vec2  SimBiController::getStanceFootPos() {
	if (nullptr !=stanceFoot_)
		return(stanceFoot_->getPosition());
	return  b2Vec2(0, 0);
}

b2Vec2  SimBiController::getSwingFootPos() {
	if (nullptr !=swingFoot_)
		return(swingFoot_->getPosition());
	return  b2Vec2(0, 0);
}

int  SimBiController::getFSMState() {
	if (FSMStateIndex_ < 0)
		setFSMStateTo(startingState_);
	return  FSMStateIndex_;
}

void SimBiController::updateDAndV(){

	b2Vec2    feetMidpoint;
	comPosition_ = character_->getCOM();

	comVelocity_ = character_->getCOMVelocity();

	d_ = VectorDelta(stanceFoot_->getPosition(), comPosition_);
	v_ = comVelocity_;

	//and now compute the vector from the COM to the center of midpoint between the feet, again expressed in world coordinates
	feetMidpoint.x = (stanceFoot_->getPosition().x + swingFoot_->getPosition().x) / 2;
	feetMidpoint.y = (stanceFoot_->getPosition().y + swingFoot_->getPosition().y) / 2;


	//now we have to compute the difference between the current COM && the desired COMPosition, in world coordinates
	doubleStanceCOMError_ = VectorDelta(comPosition_, feetMidpoint);
	//and add the user specified offset_
	doubleStanceCOMError_.y = 0;
}

void SimBiController::computeD0(double phi, b2Vec2 &d0) {
	SimBiConState* currState; 
	currState = states_[getFSMState()];
	computeDorV(phi, currState->dTrajX_, stance_, d0);
}

void SimBiController::computeV0(double phi, b2Vec2 &v0) {

	SimBiConState* currState;
	currState = states_[getFSMState()];
	computeDorV(phi, currState->vTrajX_, stance_, v0);
}

void SimBiController::computeDorV(double phi, Trajectory1d* trajX, int  stance_, b2Vec2& result) {
	result.y = 0;
	if (trajX == nullptr)
		result.x = 0;
	else
		result.x = trajX->evaluate_catmull_rom(phi);
}

//---------------------------------------TrajectoryComponent------------------------

TrajectoryComponent::TrajectoryComponent() : baseTraj_(nullptr), dTrajScale_(nullptr), vTrajScale_(nullptr),
	reverseAngleOnLeftStance_(false), reverseAngleOnRightStance_(false), offset_(0.0) {}

void TrajectoryComponent::setReverseOnStance(int stance) {
	if (stance == ROS_LEFT) {
		reverseAngleOnLeftStance_ = true;
		reverseAngleOnRightStance_ = false;
	}
	else
		if (stance == ROS_RIGHT) {
			reverseAngleOnLeftStance_ = false;
			reverseAngleOnRightStance_ = true;
		}
		else
			if (stance == ROS_DONT_REVERSE) {
				reverseAngleOnLeftStance_ = false;
				reverseAngleOnRightStance_ = false;
			}
			else {
				std::cout << "Invalid state stance\n";
				return;
			}
}

int  TrajectoryComponent::getReverseOnStance() {
	if (reverseAngleOnLeftStance_ && reverseAngleOnRightStance_) {
		std::cout << "Invalid state stance!\n";
		return -1;
	}
	if (reverseAngleOnLeftStance_)
		return(ROS_LEFT);
	if (reverseAngleOnRightStance_)
		return(ROS_RIGHT);
	return(ROS_DONT_REVERSE);
}

void TrajectoryComponent::setBaseTrajectory(Trajectory1d* traj) {
	baseTraj_ = traj;
}

Trajectory1d*  TrajectoryComponent::getBaseTrajectory() {
	return(baseTraj_);
}

void TrajectoryComponent::setVTrajScale(Trajectory1d* traj) {
	vTrajScale_ = traj;
}

Trajectory1d*  TrajectoryComponent::getVTrajScale() {
	return(vTrajScale_);
}

void TrajectoryComponent::setDTrajScale(Trajectory1d* traj) {
	dTrajScale_ = traj;
}

Trajectory1d*  TrajectoryComponent::getDTrajScale() {
	return(dTrajScale_);
}

double TrajectoryComponent::evaluateTrajectoryComponent(SimBiController *con, Joint*  j, int  stance, double  phi, const b2Vec2 d, const b2Vec2 v, bool bareTrajectory) {

	double    baseAngle;
	baseAngle = offset_;

	double scale = 1.0;
	//this d.z should really be d)tted with some axis - probably the same as the feedback one...
	if (nullptr != dTrajScale_)
		scale = scale * dTrajScale_->evaluate_linear(d.x);

	//this v.z should really be v)tted with some axis - probably the same as the feedback one...
	if (nullptr !=vTrajScale_)
		scale = scale * vTrajScale_->evaluate_linear(v.x);

	if (bareTrajectory)
		scale = 1;

	if (nullptr !=baseTraj_)
		baseAngle = baseAngle + baseTraj_->evaluate_catmull_rom(phi) * scale;

	if ((stance == LEFT_STANCE) && reverseAngleOnLeftStance_)
		baseAngle = -baseAngle;
	if ((stance == RIGHT_STANCE) && reverseAngleOnRightStance_)
		baseAngle = -baseAngle;

	return  baseAngle;
}

//------------------------------------Trajectory---------------------------

Trajectory::Trajectory() : components_(vector<TrajectoryComponent*>()),
	leftStanceIndex_(-1),
	rightStanceIndex_(-1),
	jName_("NoNameJoint"),
	relToCharFrame_(false),
	strength_Traj_(nullptr) {}

void Trajectory::setStrengthTrajectory(Trajectory1d *traj) {
	strength_Traj_ = traj;
}

Trajectory1d*  Trajectory::getStrengthTrajectory() {
	return(strength_Traj_);
}

void Trajectory::setRelativeToCharacterFrame(bool rel) {
	relToCharFrame_ = rel;
}

bool  Trajectory::isRelativeToCharacterFrame() {
	return(relToCharFrame_);
}

void Trajectory::addTrajectoryComponent(TrajectoryComponent* trajComp_disown) {
	components_.push_back(trajComp_disown);
}

void Trajectory::clearTrajectoryComponents(){
	// todo free objects
	components_.clear();
}

TrajectoryComponent*  Trajectory::getTrajectoryComponent(int index) {
	return(components_[index]);
}

int  Trajectory::getTrajectoryComponentCount() {
	return(components_.size());
}

double Trajectory::evaluateTrajectory(SimBiController *con, Joint*  j, int  stance_, double  phi, const b2Vec2 d, const b2Vec2 v, bool  bareTrajectory){

	double   a;
	int    i;
	a = 0;

	for (i = 0; i < components_.size(); ++i)
		a = components_[i]->evaluateTrajectoryComponent(con, j, stance_, phi, d, v, bareTrajectory) + a;

	return(a);
}

double  Trajectory::evaluateStrength(double phiToUse) {
	if (strength_Traj_ == nullptr) return(1.0);
	return(strength_Traj_->evaluate_catmull_rom(phiToUse));
}

int  Trajectory::getJointIndex(int stance) {
	if (stance == LEFT_STANCE) return  leftStanceIndex_;
	else return  rightStanceIndex_;
}

//---------------------------- SimBiConState--------------------------------

SimBiConState::SimBiConState() : sTraj_(vector<Trajectory*>()),
	name_("Uninitialized state"),
	nextStateIndex_(-1),
	stateTime_(0),
	reverseStance_(false),
	keepStance_(false),
	stateStance_(0),
	transitionOnFootContact_(true),
	minPhiBeforeTransitionOnFootContact_(0.5),
	minSwingFootForceForContact_(20.0),
	dTrajX_(nullptr),
	vTrajX_(nullptr){}

int  SimBiConState::getStateStance(int oldStance) {
	if (keepStance_)
		return(oldStance);
	if (!reverseStance_)
		return(stateStance_);
	if (oldStance == LEFT_STANCE)
		return  RIGHT_STANCE;
	else return  LEFT_STANCE;
}

int  SimBiConState::getTrajectoryCount() {
	return(sTraj_.size());
}

Trajectory*  SimBiConState::getTrajectory(int idx) {
	if (idx >= sTraj_.size()) return(nullptr);
	return(sTraj_[idx]);
}

Trajectory* SimBiConState::getTrajectory(const string& name){

	int    i; 
	for (i = 0; i < sTraj_.size(); ++i)
		if (sTraj_[i]->jName_ == name)
			return(sTraj_[i]);
	return  nullptr;
}

bool  SimBiConState::needTransition(double phi, double swingFoot_VerticalForce, double stanceFoot_VerticalForce) {
	//if (it is a foot contact based transition
	if (transitionOnFootContact_) {
		//transition if (we have a meaningful foot contact, && if (it)es not happen too early on...
		if (((phi > minPhiBeforeTransitionOnFootContact_) && (swingFoot_VerticalForce > minSwingFootForceForContact_)) || (phi >= 1))
			return(true);
		return(false);
	}

	//otherwise it must be a time-based transition
	if (phi >= 1)
		return(true);

	return(false);
}

void SimBiConState::setNextStateIndex(int nextStateIndex) {
	nextStateIndex_ = nextStateIndex;
}

void SimBiConState::setTransitionOnFootContact(bool transition) {
	transitionOnFootContact_ = transition;
}

bool  SimBiConState::getTransitionOnFootContact() {
	return(transitionOnFootContact_);
}

void SimBiConState::setStance(int stanceType) {
	if (stanceType == STATE_LEFT_STANCE) {
		reverseStance_ = false;
		keepStance_ = false;
		stateStance_ = LEFT_STANCE;
	}
	else
		if (stanceType == STATE_RIGHT_STANCE) {
			reverseStance_ = false;
			keepStance_ = false;
			stateStance_ = RIGHT_STANCE;
		}
		else
			if (stanceType == STATE_REVERSE_STANCE) {
				reverseStance_ = true;
				keepStance_ = false;
				stateStance_ = -1;
			}
			else
				if (stanceType == STATE_KEEP_STANCE) {
					reverseStance_ = false;
					keepStance_ = true;
					stateStance_ = -1;
				}
				else {
					std::cout << "Invalid state stance!\n";
				}
}

int  SimBiConState::getStance() {
	if (reverseStance_ && keepStance_) {
		std::cout<<"Invalid state stance!\n";
		return -1;
	}
	if (reverseStance_)
		return(STATE_REVERSE_STANCE);
	if (keepStance_)
		return(STATE_KEEP_STANCE);

	if (stateStance_ == LEFT_STANCE)
		return  STATE_LEFT_STANCE;
	else
		return  STATE_RIGHT_STANCE;
}

void SimBiConState::addTrajectory(Trajectory* traj_disown) {
	sTraj_.push_back(traj_disown);
}

void SimBiConState::clearTrajectories() {
	sTraj_.clear();
}

//-------------------- VirtualModelController-----------------------------

VirtualModelController::VirtualModelController(Character* b, const string& name)
	: Controller(b, name) {}

void VirtualModelController::computeTorques(vector<ContactPoint*>& cfs) {
	std::cout << __func__ << "don""t call this method...\n";
}

void VirtualModelController::computeJointTorquesEquivalentToForce(Joint* start, const b2Vec2 pLocal, const b2Vec2 fGlobal, Joint*  end) {
	Joint*   currentJoint;
	b2Vec2   pGlobal;
	b2Vec2   tmpV;
	double    tmpT;
	currentJoint = start;
	pGlobal = start->GetChild()->getWorldCoordinatesPoint(pLocal);

	while (currentJoint != end) {
		if (currentJoint == nullptr) {
			std::cout << "VirtualModelController::computeJointTorquesEquivalentToForce --> } was not a parent of start...";
		}
		tmpV = VectorDelta(currentJoint->GetParent()->getWorldCoordinatesPoint(currentJoint->GetParentJPos()), pGlobal);
		tmpT = b2Cross(tmpV, fGlobal);
		torques_[currentJoint->id_] = torques_[currentJoint->id_] - tmpT;
		currentJoint = currentJoint->GetParent()->parentJoint_;
	}

	//and we just have to) it once more for the end joint, if it's not NULL
	if (end != nullptr) {
		tmpV = VectorDelta(currentJoint->GetParent()->getWorldCoordinatesPoint(currentJoint->GetParentJPos()), pGlobal);
		torques_[currentJoint->id_] = torques_[currentJoint->id_] - b2Cross(tmpV, fGlobal);
	}
}

//------------------------------------IKVMCController----------------------------------

IKVMCController::IKVMCController(Character* b, const string& Name)
	: SimBiController(b, Name), 
	swingKneeIndex_(0),
	swingKnee_(nullptr), swingHip_(nullptr),
	lKneeIndex_(character_->getJointIndex("lKnee")),
	rKneeIndex_(character_->getJointIndex("rKnee")),
	lAnkleIndex_(character_->getJointIndex("lAnkle")),
	rAnkleIndex_(character_->getJointIndex("rAnkle")),
	stanceAnkleIndex_(0), stanceKneeIndex_(0), swingAnkleIndex_(0),
	vmc_(new VirtualModelController(b, "VMC")),
	ffRootTorque_(0),
	behaviour_(nullptr),//own
	velDSagittal_(0.0),
	swingFootTrajectorySagittal_(new Trajectory1d()),
	swingFootHeightTrajectory_(new Trajectory1d()),	
	doubleStanceMode_(false),
	comOffsetSagittal_(0.0), panicHeight_(0.0), unplannedForHeight_(0.0) {}

IKVMCController::~IKVMCController() {
	delete vmc_;
	delete swingFootHeightTrajectory_;
	delete swingFootTrajectorySagittal_;
	delete behaviour_;
}

void IKVMCController::setBehaviour(BehaviourController *behaviour__disown) {
	behaviour_ = behaviour__disown;
}

b2Vec2 IKVMCController::computeIPStepLocation(){

	b2Vec2   step;
	double    h;
	h = abs(comPosition_.y - stanceFoot_->getPosition().y);
	step.x = v_.x * sqrt(h / -Gravity + v_.x * v_.x / (4 * -Gravity*-Gravity)) * 1.1;
	step.y = 0;
	return  step;
}

void IKVMCController::computeIKSwingLegTargets(double dt) {

	b2Vec2    pNow; b2Vec2    pFuture; b2Vec2    parentAxis; b2Vec2    childAxis;
	pNow = getSwingFootTargetLocation(phi_, comPosition_);
	pFuture = getSwingFootTargetLocation(Min(phi_ + dt, 1), comPosition_ + b2Vec2(comVelocity_.x * dt, comVelocity_.x * dt));

	parentAxis = VectorDelta(character_->joints_[swingHipIndex_]->GetChildJPos(), character_->joints_[swingKneeIndex_]->GetParentJPos());
	childAxis = VectorDelta(character_->joints_[swingKneeIndex_]->GetChildJPos(), character_->joints_[swingAnkleIndex_]->GetParentJPos());

	computeIKQandW(swingHipIndex_, swingKneeIndex_, parentAxis, swingLegPlaneOfRotation, 1, childAxis, pNow, true, pFuture, dt);
}

void IKVMCController::DebugComputeIKSwingLegTargets(b2Vec2 const Point) {

	b2Vec2    pNow; b2Vec2    pFuture; b2Vec2    parentAxis; b2Vec2    childAxis; 
	pNow = Point;
	pFuture = Point;
	parentAxis = VectorDelta(character_->joints_[swingHipIndex_]->GetChildJPos(), character_->joints_[swingKneeIndex_]->GetParentJPos());
	childAxis = VectorDelta(character_->joints_[swingKneeIndex_]->GetChildJPos(), character_->joints_[swingAnkleIndex_]->GetParentJPos());

	computeIKQandW(swingHipIndex_, swingKneeIndex_, parentAxis, swingLegPlaneOfRotation, 1, childAxis, pNow, false, pFuture, 0.001);
}


void IKVMCController::computeTorques(vector<ContactPoint*>& cfs) {
	evaluateJointTargets();
	//now overwrite the target angles for (the swing hip && the swing knee in order to ensure foot-placement control
	if (!doubleStanceMode_)
		computeIKSwingLegTargets(0.001);
	computePDTorques(cfs);

	bubbleUpTorques();

	computeGravityCompensationTorques();

	ffRootTorque_ = 0;

	if (cfs.size() > 0)
		COMJT(cfs);

	if (doubleStanceMode_ && (cfs.size() > 0))
		computeLegTorques(swingAnkleIndex_, swingKneeIndex_, swingHipIndex_, cfs);

	computeHipTorques(aRootD_, getStanceFootWeightRatio(cfs), ffRootTorque_);

}

b2Vec2 IKVMCController::getSwingFootTargetLocation(double t, b2Vec2  const com) {
	b2Vec2    step; 
	step.x = swingFootTrajectorySagittal_->evaluate_catmull_rom(t);
	step.y = 0;
	//add it to the com location
	step = com + step;
	//finally, set the desired height of the foot
	step.y = swingFootHeightTrajectory_->evaluate_catmull_rom(t) + panicHeight_ + unplannedForHeight_;

	return  step;
}

void IKVMCController::computeGravityCompensationTorques() {

	int    i; 
	vmc_->resetTorques();
	for (i = 0; i < character_->joints_.size(); ++i) {
		if ((i != stanceHipIndex_) && (i != stanceKneeIndex_) && (i != stanceAnkleIndex_))
			vmc_->computeJointTorquesEquivalentToForce(character_->joints_[i], b2Vec2(0, 0), b2Vec2(0, character_->joints_[i]->GetChild()->getMass()*-Gravity), nullptr);
	}

	for (i = 0; i < character_->joints_.size(); ++i)
		torques_[i] = torques_[i] + vmc_->torques_[i];
}

void IKVMCController::updateSwingAndStanceReferences() {
	stanceHipIndex_ = stance_ == LEFT_STANCE ? lHipIndex_ : rHipIndex_;
	swingHipIndex_ = stance_ == LEFT_STANCE ? rHipIndex_ : lHipIndex_;
	stanceKneeIndex_ = stance_ == LEFT_STANCE ? lKneeIndex_ : rKneeIndex_;
	swingKneeIndex_ = stance_ == LEFT_STANCE ? rKneeIndex_ : lKneeIndex_;
	stanceAnkleIndex_ = stance_ == LEFT_STANCE ? lAnkleIndex_ : rAnkleIndex_;
	swingAnkleIndex_ = stance_ == LEFT_STANCE ? rAnkleIndex_ : lAnkleIndex_;

	swingHip_ = character_->joints_[swingHipIndex_];
	swingKnee_ = character_->joints_[swingKneeIndex_];
}

void boundToRange(double &v, double  min, double  max) {
	if (v < min)
		v = min;
	else
		if (v > max)
			v = max;
}

void IKVMCController::computeIKQandW(int parentJIndex, int childJIndex, const b2Vec2 parentAxis, double  parentNormal,
	double childNormal, const b2Vec2 childEndEffector, const b2Vec2 wP, bool computeAngVelocities, const b2Vec2 futureWP, double dt){

	CharacterState*   rs;
	Joint*   parentJoint;
	ArticulatedRigidBody*   gParent;
	double aParent, aChild;
	double   wParentD; double   wChildD;
	b2Vec2   velOffset;
	double aParentF, aChildF;
	double    aDiff; double    aDiffv;
	rs = desiredPose_;

	parentJoint = character_->joints_[parentJIndex];
	gParent = parentJoint->GetParent();

	TwoLinkIK::getIKOrientations(parentJoint->GetParentJPos(), gParent->getLocalCoordinatesPoint(wP), parentNormal, parentAxis,
		childNormal, childEndEffector, aParent, aChild);

	controlParams_[parentJIndex]->relToFrame_ = false;
	controlParams_[childJIndex]->relToFrame_ = false;
	rs->setJointRelativeOrientation(aParent, parentJIndex);
	rs->setJointRelativeOrientation(aChild, childJIndex);

	wParentD = 0;
	wChildD = 0;

	if (computeAngVelocities) {
		//the joint"s origin will also move, so take that into account, by subbing the offset_ by which it moves to the
		//futureTarget (to get the same relative position to the hip)
		velOffset = gParent->getAbsoluteVelocityForLocalPoint(parentJoint->GetParentJPos());

		TwoLinkIK::getIKOrientations(parentJoint->GetParentJPos(), gParent->getLocalCoordinatesPoint(futureWP + b2Vec2(velOffset.x * -dt, velOffset.y * -dt)),
			parentNormal, parentAxis, childNormal, childEndEffector, aParentF, aChildF);

		aDiff = aParentF + -aParent;
		aDiffv = sin(aDiff / 2);
		wParentD = aDiffv * 2 / dt;
		//the above is the desired angular velocity, were the parent not rotating already - but it usually is, so we"ll account for (that
		wParentD = wParentD - gParent->getAngularVelocity();

		aDiff = aChildF + -aChild;
		aDiffv = sin(aDiff / 2);
		wChildD = aDiffv * 2 / dt;

		//make sure we)n"t go overboard with the estimates, in case there are discontinuities in the trajectories...
		boundToRange(wChildD, -5.0, 5.0);
		boundToRange(wParentD, -5.0, 5.0);
	}

	rs->setJointRelativeAngVelocity(wParentD, parentJIndex);
	rs->setJointRelativeAngVelocity(wChildD, childJIndex);
}

void IKVMCController::bubbleUpTorques() {
	int    i; 
	for (i = character_->joints_.size() - 1; i >= 0; --i) {
		if ((i != stanceHipIndex_) && (i != stanceKneeIndex_))
			if (character_->joints_[i]->GetParent() != root_)
				torques_[character_->joints_[i]->GetParent()->parentJoint_->id_] += torques_[i];
	}
}

void IKVMCController::computeLegTorques(int ankleIndex, int kneeIndex, int hipIndex, vector<ContactPoint*>&  cfs) {

	b2Vec2   fA; b2Vec2   p; b2Vec2   r;
	double   ankleTorque;
	int    lBackIndex; int    mBackIndex;
	fA = computeVirtualForce();
	p = comPosition_;

	r = VectorDelta(character_->joints_[ankleIndex]->GetChild()->getWorldCoordinatesPoint(character_->joints_[ankleIndex]->GetChildJPos()), p);

	ankleTorque = b2Cross(r, fA);
	preprocessAnkleVTorque(ankleIndex, cfs, ankleTorque);
	torques_[ankleIndex] = torques_[ankleIndex] + ankleTorque;

	r = VectorDelta(character_->joints_[kneeIndex]->GetChild()->getWorldCoordinatesPoint(character_->joints_[kneeIndex]->GetChildJPos()), p);
	torques_[kneeIndex] = torques_[kneeIndex] + b2Cross(r, fA);

	r = VectorDelta(character_->joints_[hipIndex]->GetChild()->getWorldCoordinatesPoint(character_->joints_[hipIndex]->GetChildJPos()), p);
	torques_[hipIndex] = torques_[hipIndex] + b2Cross(r, fA);

	//the torque on the stance hip is cancelled out, so pass it in as a torque that the root wants to see!
	ffRootTorque_ = ffRootTorque_ - b2Cross(r, fA);

	lBackIndex = character_->getJointIndex("pelvis_lowerback");
	r = VectorDelta(character_->joints_[lBackIndex]->GetChild()->getWorldCoordinatesPoint(character_->joints_[lBackIndex]->GetChildJPos()), p);
	torques_[lBackIndex] = torques_[lBackIndex] + b2Cross(r, fA) / 10;

	mBackIndex = character_->getJointIndex("lowerback_torso");
	r = VectorDelta(character_->joints_[mBackIndex]->GetChild()->getWorldCoordinatesPoint(character_->joints_[mBackIndex]->GetChildJPos()), p);
	torques_[mBackIndex] = torques_[mBackIndex] + b2Cross(r, fA) / 10;
}

inline b2Vec2 Mul(b2Vec2 v2, double p) {
	v2.x *= p;
	v2.y *= p;
	return v2;
}

void IKVMCController::COMJT(vector<ContactPoint*>& cfs) {

	int   lBackIndex; int   mBackIndex;
	double   m;
	ArticulatedRigidBody*   tibia; ArticulatedRigidBody*   femur; ArticulatedRigidBody*   pelvis; ArticulatedRigidBody*   lBack; ArticulatedRigidBody*   mBack;
	b2Vec2 anklePos, kneePos, hipPos, lbackPos, mbackPos;
	b2Vec2   fA; b2Vec2   f1; b2Vec2   f2; b2Vec2   f3; b2Vec2   f4; b2Vec2   f5;
	double    ankleTorque;
	lBackIndex = character_->getJointIndex("pelvis_lowerback");
	mBackIndex = character_->getJointIndex("lowerback_torso");

	tibia = character_->joints_[stanceAnkleIndex_]->GetParent();
	femur = character_->joints_[stanceKneeIndex_]->GetParent();
	pelvis = character_->joints_[stanceHipIndex_]->GetParent();
	lBack = character_->joints_[lBackIndex]->GetChild();;
	mBack = character_->joints_[mBackIndex]->GetChild();;

	anklePos = character_->joints_[stanceAnkleIndex_]->GetChild()->getWorldCoordinatesPoint(character_->joints_[stanceAnkleIndex_]->GetChildJPos());
	kneePos = character_->joints_[stanceKneeIndex_]->GetChild()->getWorldCoordinatesPoint(character_->joints_[stanceKneeIndex_]->GetChildJPos());
	hipPos = character_->joints_[stanceHipIndex_]->GetChild()->getWorldCoordinatesPoint(character_->joints_[stanceHipIndex_]->GetChildJPos());
	lbackPos = character_->joints_[lBackIndex]->GetChild()->getWorldCoordinatesPoint(character_->joints_[lBackIndex]->GetChildJPos());
	mbackPos = character_->joints_[mBackIndex]->GetChild()->getWorldCoordinatesPoint(character_->joints_[mBackIndex]->GetChildJPos());

	//total mass...
	m = tibia->getMass() + femur->getMass() + pelvis->getMass() + lBack->getMass() + mBack->getMass();

	fA = computeVirtualForce();

	f1 = VectorDelta(anklePos, Mul(tibia->getPosition(), tibia->getMass())) +
		VectorDelta(anklePos, Mul(femur->getPosition(), femur->getMass())) +
		VectorDelta(anklePos, Mul(pelvis->getPosition(), pelvis->getMass())) +
		VectorDelta(anklePos, Mul(lBack->getPosition(), lBack->getMass())) +
		VectorDelta(anklePos, Mul(mBack->getPosition(), mBack->getMass()));
	f1.x = f1.x / m;
	f1.y = f1.y / m;

	f2 = VectorDelta(kneePos, Mul(femur->getPosition(), femur->getMass())) +
		VectorDelta(kneePos, Mul(pelvis->getPosition(), pelvis->getMass())) +
		VectorDelta(kneePos, Mul(lBack->getPosition(), lBack->getMass())) +
		VectorDelta(kneePos, Mul(mBack->getPosition(), mBack->getMass()));
	f2.x = f2.x / m;
	f2.y = f2.y / m;

	f3 = VectorDelta(hipPos, Mul(pelvis->getPosition(), pelvis->getMass())) +
		VectorDelta(hipPos, Mul(lBack->getPosition(), lBack->getMass())) +
		VectorDelta(hipPos, Mul(mBack->getPosition(), mBack->getMass()));
	f3.x = f3.x / m;
	f3.y = f3.y / m;

	f4 = VectorDelta(lbackPos, Mul(lBack->getPosition(), lBack->getMass())) +
		VectorDelta(lbackPos, Mul(mBack->getPosition(), mBack->getMass()));
	f4.x = f4.x / m;
	f4.y = f3.y / m;

	f5 = VectorDelta(mbackPos, Mul(mBack->getPosition(), mBack->getMass()));
	f5.x = f5.x / m;
	f5.y = f5.y / m;

	ankleTorque = b2Cross(f1, fA);
	preprocessAnkleVTorque(stanceAnkleIndex_, cfs, ankleTorque);

	torques_[stanceAnkleIndex_] = torques_[stanceAnkleIndex_] + ankleTorque;
	torques_[stanceKneeIndex_] = torques_[stanceKneeIndex_] + b2Cross(f2, fA);
	torques_[stanceHipIndex_] = torques_[stanceHipIndex_] + b2Cross(f3, fA);

	//the torque on the stance hip is cancelled out, so pass it in as a torque that the root wants to see!
	ffRootTorque_ = ffRootTorque_ - b2Cross(f3, fA);

	torques_[lBackIndex] = torques_[lBackIndex] - b2Cross(f4, fA) * 0.5;
	torques_[mBackIndex] = torques_[mBackIndex] - b2Cross(f5, fA) * 0.3;
}

b2Vec2 IKVMCController::computeVirtualForce() {

	b2Vec2    desA; b2Vec2    errV; b2Vec2    fA;
	//this is the desired acceleration of the center of mass
	desA.x = (velDSagittal_ - v_.x) * 30;
	desA.y = 0;

	if (doubleStanceMode_) {
		errV = b2Vec2(0,0) - doubleStanceCOMError_;
		desA.x = (-errV.x + comOffsetSagittal_) * 10 + (velDSagittal_ - v_.x) * 150;
	}

	//and this is the force that would achieve that - make sure it"s not too large...
	double tmp = desA.x * character_->mass_;
	fA.y = desA.y * character_->mass_;
	boundToRange(tmp, -60, 60);
	fA.x = tmp;

	return  fA;
}

void IKVMCController::preprocessAnkleVTorque(int ankleJointIndex, vector<ContactPoint*>&  cfs, double& ankleVTorque) {
	
	bool   heelInContact; bool   toeInContact;
	ArticulatedRigidBody*   foot;
	double    footRelativeAngularVel;
	foot = character_->joints_[ankleJointIndex]->GetChild();;
	getForceInfoOn(foot, cfs, heelInContact, toeInContact);

	if (!toeInContact || (phi_ < 0.2) || (phi_ > 0.8)) ankleVTorque = 0;

	footRelativeAngularVel = foot->getAngularVelocity();

	if (abs(footRelativeAngularVel) > 1.0) ankleVTorque = 0;
}

void IKVMCController::performPreTasks(double dt, vector<ContactPoint*>&  cfs) {
	if (nullptr != behaviour_)
		behaviour_->simStepPlan(dt);
	SimBiController::performPreTasks(dt, cfs);
}

bool IKVMCController::performPostTasks(double dt, vector<ContactPoint*>&  cfs) {

	bool    newState; 
	newState = SimBiController::performPostTasks(dt, cfs);
	if (newState)
		if (nullptr !=behaviour_)
			behaviour_->conTransitionPlan();

	return(newState);
}

void IKVMCController::getForceInfoOn(ArticulatedRigidBody* rb, vector<ContactPoint*>&  cfs, bool& heelForce, bool& toeForce){

	int   i;
	b2Vec2    tmpP;
	//figure out if (the toe/heel are in contact...
	heelForce = false;
	toeForce = false;
	for (i = 0; i < cfs.size(); ++i) {
		if (haveRelationBetween(cfs[i]->rb1, rb) || haveRelationBetween(cfs[i]->rb2, rb)) {
			tmpP = rb->getLocalCoordinatesPoint(cfs[i]->cp);
			if (tmpP.x < 0) heelForce = true;
			if (tmpP.x > 0) toeForce = true;
		}
	}
}

//-------------------------------BehaviourController--------------------------

void BehaviourController::setUpperBodyPose(double leanSagittal) {

	SimBiConState   *curState;
	Trajectory  *tmpTraj; 
	curState = lowLCon_->states_[lowLCon_->getFSMState()];
	tmpTraj = curState->getTrajectory("root");
	if (nullptr !=tmpTraj)
		tmpTraj->components_[0]->offset_ = leanSagittal;
	tmpTraj = curState->getTrajectory("pelvis_lowerback");
	if (nullptr !=tmpTraj)
		tmpTraj->components_[0]->offset_ = leanSagittal * 1.5;
	tmpTraj = curState->getTrajectory("lowerback_torso");
	if (nullptr !=tmpTraj)
		tmpTraj->components_[0]->offset_ = leanSagittal * 2.5;
	tmpTraj = curState->getTrajectory("torso_head");
	if (nullptr !=tmpTraj)
		tmpTraj->components_[0]->offset_ = leanSagittal * 3.0;
}

void BehaviourController::setKneeBend(double v, bool swingAlso) {

	SimBiConState* curState;
	Trajectory* tmpTraj; 
	curState = lowLCon_->states_[lowLCon_->FSMStateIndex_];

	tmpTraj = curState->getTrajectory("STANCE_Knee");
	tmpTraj->components_[0]->offset_ = v;

	if (swingAlso) {
		tmpTraj = curState->getTrajectory("SWING_Knee");
		tmpTraj->components_[0]->offset_ = v;
	}
}

void BehaviourController::setVelocities(double velDS) {
	lowLCon_->velDSagittal_ = velDS;
}

BehaviourController::BehaviourController(Character* b, IKVMCController* llc, WorldOracle*  w)
  : swingFootStartPos_(0,0) {
	bip_ = b;
	lowLCon_ = llc;
	wo_ = w;

	//we should estimate these from the character info...	
	ubSagittalLean_ = 0,
	kneeBend_ = 0;
	stepHeight_ = 0;

	legLength_ = 1;
	ankleBaseHeight_ = 0.04;

	stepTime_ = 0.6;
}

BehaviourController::~BehaviourController() {
	delete wo_;
}

void BehaviourController::adjustStepHeight() {

	double    panicIntensity; double    foot_sole_y; 
	lowLCon_->unplannedForHeight_ = 0;
	if (nullptr != wo_) {
		//the trajectory of the foot was generated without taking the environment into account, so check to see if (there are any un-planned bumps (at somepoint in the near future)
		b2Vec2 swingFootVelocity = lowLCon_->swingFoot_->getVelocity();
		lowLCon_->unplannedForHeight_ = wo_->getWorldHeightAt(lowLCon_->swingFoot_->getPosition() + b2Vec2(swingFootVelocity.x * 0.3, swingFootVelocity.y * 0.3)) * 1.0;
		foot_sole_y = lowLCon_->swingFoot_->getWorldCoordinatesPoint(b2Vec2(0, -lowLCon_->swingFoot_->height_ / 2)).y;
		lowLCon_->unplannedForHeight_ = lowLCon_->unplannedForHeight_ + (lowLCon_->unplannedForHeight_ - foot_sole_y) * 0.5;
		WorldOracle::debug_point_.y = lowLCon_->unplannedForHeight_;
	}

	//if (the foot is high enough, we shouldn"t) much about it... also, if (we"re close to the start || } of the
	//walk cycle, we)n"t need to) anything... the thing below is a quadratic that is 1 at 0.5, 0 at 0 && 1...
	panicIntensity = -4 * lowLCon_->phi_ * lowLCon_->phi_ + 4 * lowLCon_->phi_;
	panicIntensity = panicIntensity * getPanicLevel();
	lowLCon_->panicHeight_ = panicIntensity * 0.05;
}

void BehaviourController::setElbowAngles(double leftElbowAngle, double rightElbowAngle) {

	double   stanceElbowAngle; double   swingElbowAngle;
	SimBiConState   *curState;
	Trajectory  *tmpTraj;
	stanceElbowAngle = lowLCon_->stance_ == LEFT_STANCE ? leftElbowAngle : rightElbowAngle;
	swingElbowAngle = lowLCon_->stance_ == LEFT_STANCE?  rightElbowAngle : leftElbowAngle;

	curState = lowLCon_->states_[lowLCon_->FSMStateIndex_];
	tmpTraj = curState->getTrajectory("STANCE_Elbow");
	if (nullptr !=tmpTraj)
		tmpTraj->components_[0]->offset_ = stanceElbowAngle;
	tmpTraj = curState->getTrajectory("SWING_Elbow");
	if (nullptr !=tmpTraj)
		tmpTraj->components_[0]->offset_ = swingElbowAngle * -1;
}

void BehaviourController::setShoulderAngles(double leftSwing, double rightSwing) {

	double   stanceSwing; double   swingSwing;
	SimBiConState   *curState;
	Trajectory    *tmpTraj; 
	stanceSwing = lowLCon_->stance_ == LEFT_STANCE? leftSwing : rightSwing;
	swingSwing = lowLCon_->stance_ == RIGHT_STANCE ? leftSwing : rightSwing;

	curState = lowLCon_->states_[lowLCon_->FSMStateIndex_];
	tmpTraj = curState->getTrajectory("STANCE_Shoulder");
	if (nullptr !=tmpTraj)
		tmpTraj->components_[0]->offset_ = stanceSwing;

	tmpTraj = curState->getTrajectory("SWING_Shoulder");
	if (nullptr !=tmpTraj)
		tmpTraj->components_[0]->offset_ = swingSwing;
}

void BehaviourController::requestStepTime(double stepTime) {
	stepTime_ = stepTime;
}

void BehaviourController::requestStepHeight(double stepHeight) {
	stepHeight_ = stepHeight;
}

void BehaviourController::requestVelocities(double velDS) {
	velDSagittal_ = velDS;
}

void BehaviourController::requestUpperBodyPose(double leanS) {
	ubSagittalLean_ = leanS;
}

void BehaviourController::requestKneeBend(double kb) {
	kneeBend_ = kb;
}

void BehaviourController::setDesiredSwingFootLocation() {
	const double dt = 0.001;

	b2Vec2    step; 
	step = computeSwingFootLocationEstimate(lowLCon_->comPosition_, lowLCon_->phi_);
	lowLCon_->swingFootTrajectorySagittal_->setKnotValue(0, step.x);

	b2Vec2 comVelocity = lowLCon_->comVelocity_;

	step = computeSwingFootLocationEstimate(lowLCon_->comPosition_ + b2Vec2(comVelocity.x * dt, comVelocity.y * dt), lowLCon_->phi_ + dt);
	lowLCon_->swingFootTrajectorySagittal_->setKnotValue(1, step.x);
	//to give some gradient information, here"s what the position will be a short time later...

	lowLCon_->swingFootTrajectorySagittal_->setKnotPosition(0, lowLCon_->phi_);
	lowLCon_->swingFootTrajectorySagittal_->setKnotPosition(1, lowLCon_->phi_ + dt);
}

b2Vec2 BehaviourController::computeSwingFootLocationEstimate(b2Vec2 const comPos, double  phase) { 

	b2Vec2   step; b2Vec2   initialStep;
	double    t;
	step = lowLCon_->computeIPStepLocation();

	double tmp = step.x - lowLCon_->velDSagittal_ / 20;
	boundToRange(tmp, -0.4 * legLength_, 0.4 * legLength_);
	step.x = tmp;

	initialStep = VectorDelta(comPos, swingFootStartPos_);
	//when phi is small, we want to be much closer to where the initial step is - so compute this quantity in character-relative coordinates
	//now interpolate between this position && initial foot position - but provide two estimates in order to provide some gradient information
	t = (1 - phase);
	t = t * t;
	boundToRange(t, 0, 1);

	b2Vec2 result = b2Vec2(0, 0);
	result.x = result.x + step.x * (1 - t);
	result.x = result.x + initialStep.x * t;
	result.y = 0;
	return result;
}

void BehaviourController::initializeDefaultParameters() {
	lowLCon_->updateDAndV();
}

void BehaviourController::simStepPlan(double dt) {
	lowLCon_->updateSwingAndStanceReferences();
	if (lowLCon_->phi_ <= 0.01)
		swingFootStartPos_ = lowLCon_->swingFoot_->getWorldCoordinatesPoint(bip_->joints_[lowLCon_->swingAnkleIndex_]->GetChildJPos());

	//compute desired swing foot location...
	setDesiredSwingFootLocation();

	//set some of these settings
	setUpperBodyPose(ubSagittalLean_);
	setKneeBend(kneeBend_);
	setVelocities(velDSagittal_);

	//adjust for (panic mode || unplanned terrain...
	adjustStepHeight();
}

void BehaviourController::conTransitionPlan() {
	lowLCon_->updateSwingAndStanceReferences();
	lowLCon_->updateDAndV();
	lowLCon_->states_[0]->stateTime_ = stepTime_;

	lowLCon_->updateSwingAndStanceReferences();
	swingFootStartPos_ = lowLCon_->swingFoot_->getPosition();

	//now prepare the step information for (the following step;
	lowLCon_->swingFootHeightTrajectory_->clear();
	lowLCon_->swingFootTrajectorySagittal_->clear();

	lowLCon_->swingFootHeightTrajectory_->addKnot(0, ankleBaseHeight_);
	lowLCon_->swingFootHeightTrajectory_->addKnot(0.5, ankleBaseHeight_ + 0.01 + 0.1 + 0 + stepHeight_);
	lowLCon_->swingFootHeightTrajectory_->addKnot(1, ankleBaseHeight_ + 0.01);
	
	lowLCon_->swingFootTrajectorySagittal_->addKnot(0,0);
	lowLCon_->swingFootTrajectorySagittal_->addKnot(1,0);
}

double  getValueInFuzzyRange(double val, double minB, double minG, double maxG, double maxB) {
	if ((val <= minB) || (val >= maxB))
		return(1);
	if ((val >= minG) && (val <= maxG))
		return(0);
	if ((val > minB) && (val < minG))
		return((minG - val) / (minG - minB));
	if ((val > maxG) && (val < maxB))
		return((val - maxG) / (maxB - maxG));
	//the input was probably wrong, so return panic...
	return(1);
}

double BehaviourController::getPanicLevel() {

	double    panicEstimate;
	panicEstimate = getValueInFuzzyRange(lowLCon_->v_.x, lowLCon_->velDSagittal_ - 0.4, lowLCon_->velDSagittal_ - 0.3, lowLCon_->velDSagittal_ + 0.3, lowLCon_->velDSagittal_ + 0.4);
	return  panicEstimate / 1.0;
}
