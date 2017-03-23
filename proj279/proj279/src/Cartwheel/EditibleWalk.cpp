#pragma once
#include "Cartwheel/EditibleWalk.h"

using namespace std;

EditableWalking::EditableWalking(Character *cha) : IKVMCController(cha, "Editable Walking") {	

	stanceHipDamping_ = 25;
	stanceHipMaxVelocity_ = 4;

	addControlParams2("root", 1000.0, 200.0, 200.0, 1.0);
	addControlParams2("pelvis_lowerback", 75.0, 17.0, 100.0, 1.0);
	addControlParams2("lowerback_torso", 75.0, 17.0, 100.0, 1.0);
	addControlParams2("torso_head", 10.0, 3.0, 200.0, 1.0);
	addControlParams2("lShoulder", 15.0, 5.0, 200.0, 1.0);
	addControlParams2("rShoulder", 15.0, 5.0, 200.0, 1.0);
	addControlParams2("lElbow", 5.0, 1.0, 200.0, 1.0);
	addControlParams2("rElbow", 5.0, 1.0, 200.0, 1.0);
	addControlParams2("lHip", 300.0, 35.0, 200.0, 1.0);
	addControlParams2("rHip", 300.0, 35.0, 200.0, 1.0);
	addControlParams2("lKnee", 300.0, 35.0, 1000.0, 1.0);
	addControlParams2("rKnee", 300.0, 35.0, 1000.0, 1.0);
	addControlParams2("lAnkle", 50.0, 15.0, 100.0, 1.0);
	addControlParams2("rAnkle", 50.0, 15.0, 100.0, 1.0);
	addControlParams2("lToeJoint", 2.0, 0.2, 100.0, 1.0);
	addControlParams2("rToeJoint", 2.0, 0.2, 100.0, 1.0);

	SimBiConState *state = new SimBiConState();
	// defaults
	state->setTransitionOnFootContact(true);
	state->setStance(SimBiConState::STATE_REVERSE_STANCE);
	state->stateTime_ = 0.5;
	// set
	state->name_ = "State 0";
	state->nextStateIndex_ = 0;
	state->stateTime_ = 0.6;

	addTrajectory(state, "STANCE_Knee", {0.0, 0.0});
	addTrajectory(state, "SWING_Ankle", {0.0, 0.0}, true);
	addTrajectory(state, "STANCE_Ankle", {0.0, 0.0}, true);
	addTrajectory(state, "SWING_Shoulder", {0.0, 0.0}, true);
	addTrajectory(state, "STANCE_Shoulder", {0.0, 0.0}, true);
	addTrajectory(state, "pelvis_lowerback", {0.0, 0.0}, true);
	addTrajectory(state, "lowerback_torso", {0.0, 0.0}, true);
	addTrajectory(state, "torso_head", {0.0, 0.0}, true);
	addTrajectory(state, "SWING_ToeJoint", {0.0, 0.0});
	addTrajectory(state, "STANCE_ToeJoint", {0.0, 0.0});

	addState(state);
}

void EditableWalking::addControlParams2(const string& jointname, double kp, double kd, double tauMax, double scale) {
	Joint * joint  = character_->getJointByName(jointname);
	ControlParams *params = new ControlParams(joint);
	params->kp_ = kp;
	params->kd_ = kd;
	params->maxAbsTorque_ = tauMax;
	params->scale_ = scale;
	addControlParams(params);
}

void EditableWalking::addTrajectory(SimBiConState *state, const string jointname,
	const vector<double>& BaseTrajectoryArray, bool relToCharFrame, int reverseOnStance) {	
	Trajectory1d *baseTraj = new Trajectory1d();
	int i = 0;
	while (i < BaseTrajectoryArray.size()) {
		baseTraj->addKnot(BaseTrajectoryArray[i], BaseTrajectoryArray[i + 1]);
		i += 2;
	}

	Trajectory *traj = new Trajectory();
	traj->setRelativeToCharacterFrame(relToCharFrame);
	traj->jName_ = jointname;

	TrajectoryComponent *trajComp = new TrajectoryComponent();
	trajComp->setReverseOnStance(reverseOnStance);
	trajComp->setBaseTrajectory(baseTraj);
	traj->addTrajectoryComponent(trajComp);

	state->addTrajectory(traj);
}