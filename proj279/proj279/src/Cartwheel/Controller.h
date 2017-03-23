#pragma once
#include "Box2D.h"
#include "Cartwheel/Character.h"
#include "Cartwheel/WorldOracle.h"
#include "Cartwheel/Trajectory.h"
#include <vector>
#include <string>
using std::vector;
using std::string;

class SimBiController;
class IKVMCController;

#define LEFT_STANCE 0
#define RIGHT_STANCE 1
#define DONT_REVERSE 100

const b2Vec2 Up(0.0, 1.0);
const double Gravity = -9.8;

class WorldOracle;

class Controller {
public:
	Controller(Character *ch, const string& name);
	virtual ~Controller() {}
	int      GetJointCount();
	virtual void performPreTasks(double dt, vector<ContactPoint*>& cfs);
	virtual bool performPostTasks(double dt, vector<ContactPoint*>& cfs);
	virtual void computeTorques(vector<ContactPoint*>& cfs) = 0;
	void applyTorques();
	void resetTorques();

	vector<double>     oldTorques_, torques_;
	bool     noOldTorqueInfo;
	string    name_;
	Character *character_;
};

class ControlParams {
public:
	ControlParams(Joint *joint = nullptr);
	string getJointName();
	Joint *getJoint();

	Joint *joint_;
	bool controlled_;
	double kp_, kd_, maxAbsTorque_, scale_, strength_;
	bool qRelExternallyComputed_, relToFrame_;
	ControlParams() = delete;
};

struct JointRelative {
	double  orientation_, angularVelocity_;
	JointRelative() : orientation_(0.0), angularVelocity_(0.0) {}
};

class CharacterState {
public:
	CharacterState(int jointCount);
	b2Vec2 position_, velocity_;
	double orientation_, angularVelocity_;
	vector<JointRelative>     jointRelatives_;
	double getJointRelativeOrientation(int jIndex);
	void setJointRelativeOrientation(double a, int  jIndex);
	double getJointRelativeAngVelocity(int jIndex);
	void setJointRelativeAngVelocity(double w, int  jIndex);
	CharacterState() = delete;
};

class PoseController : public Controller {
public:
	PoseController(Character *ch, const string &name);
	virtual ~PoseController();

	CharacterState *desiredPose_;
	vector<ControlParams*> controlParams_;

	virtual void addControlParams(ControlParams *params);
	virtual void scaleGains(double factor);
	double computePDTorque(double aRel, double aRelD, double wRel, double wRelD, ControlParams *cParams);
	void limitTorque(double &torque, ControlParams *cParams);
	void scaleAndLimitTorque(double &torque, ControlParams *cParams);
	void computeTorques(vector<ContactPoint*>& cfs) override;
};

class TrajectoryComponent {
public:
	const int ROS_LEFT = LEFT_STANCE;
	const int ROS_RIGHT = RIGHT_STANCE;
	const int ROS_DONT_REVERSE = DONT_REVERSE;

	TrajectoryComponent();
	void setReverseOnStance(int stance);
	int  getReverseOnStance();

	void setBaseTrajectory(Trajectory1d* traj);
	Trajectory1d *getBaseTrajectory();

	void setVTrajScale(Trajectory1d* traj);
	Trajectory1d *getVTrajScale();

	void setDTrajScale(Trajectory1d* traj);
	Trajectory1d *getDTrajScale();

	double evaluateTrajectoryComponent(SimBiController *con, Joint*  j,
		int  stance, double  phi, const b2Vec2 d, const b2Vec2 v,
		bool bareTrajectory = false);

	Trajectory1d *baseTraj_, *dTrajScale_, *vTrajScale_;
	bool reverseAngleOnLeftStance_, reverseAngleOnRightStance_;
	double offset_;
};

class Trajectory {
public:	
	Trajectory();
	void setStrengthTrajectory(Trajectory1d *traj);
	Trajectory1d *getStrengthTrajectory();
	void setRelativeToCharacterFrame(bool rel = true);
	bool isRelativeToCharacterFrame();

	void addTrajectoryComponent(TrajectoryComponent *trajComp_disown);
	void clearTrajectoryComponents();
	TrajectoryComponent *getTrajectoryComponent(int index);
	int getTrajectoryComponentCount();

	double evaluateTrajectory(SimBiController *con, Joint  *j, int  stance,
		double  phi, b2Vec2  const d, b2Vec2 v, bool bareTrajectory = false);
	double evaluateStrength(double phiToUse);
	int getJointIndex(int stance);


	vector<TrajectoryComponent*> components_;
	int leftStanceIndex_, rightStanceIndex_;
	string jName_;
	bool  relToCharFrame_;
	Trajectory1d *strength_Traj_;
};

class SimBiConState {
public:
	static const int STATE_LEFT_STANCE = LEFT_STANCE;
	static const int STATE_RIGHT_STANCE = RIGHT_STANCE;
	static const int STATE_REVERSE_STANCE = 100;
	static const int STATE_KEEP_STANCE = 101;
	
	SimBiConState();
	int getStateStance(int oldStance);
	int getTrajectoryCount();
	Trajectory *getTrajectory(int idx);
	Trajectory *getTrajectory(const string& name);

	bool needTransition(double phi, double swingFootVerticalForce,
		double stanceFootVerticalForce);
	void setNextStateIndex(int nextStateIndex);
	void setTransitionOnFootContact(bool transition = true);
	bool getTransitionOnFootContact();
	void setStance(int stanceType);
	int  getStance();
	void addTrajectory(Trajectory *traj_disown);
	void clearTrajectories();


	vector<Trajectory*> sTraj_;
	string name_;
	int nextStateIndex_;
	double stateTime_;
	bool reverseStance_, keepStance_;
	int stateStance_;
	bool transitionOnFootContact_;
	double minPhiBeforeTransitionOnFootContact_, minSwingFootForceForContact_;
	Trajectory1d *dTrajX_, *vTrajX_;
};

class SimBiController : public PoseController {
public:
	SimBiController(Character *b, const string& name = "UnnamedSimBiController");
	virtual ~SimBiController();

	void scaleGains(double factor) override;
	void addControlParams(ControlParams *params) override;
	SimBiConState *getCurrentState();
	double *getPhiPtr();
	void setFSMStateTo(int index);
	void transitionToState(int stateIndex);
	b2Vec2 getForceOn(ArticulatedRigidBody* rb, vector<ContactPoint*>&  cfs);
	b2Vec2 getForceOnFoot(ArticulatedRigidBody* foot, vector<ContactPoint*>&  cfs);
	bool haveRelationBetween(ArticulatedRigidBody* rb, ArticulatedRigidBody* whichBody);
	bool isFoot(ArticulatedRigidBody* rb);
	bool isStanceFoot(ArticulatedRigidBody* rb);
	bool isSwingFoot(ArticulatedRigidBody* rb);
	double getStanceFootWeightRatio(vector<ContactPoint*>& cfs);
	void computeHipTorques(double aRootD, double stanceHipToSwingHipRatio, double ffRootTorque);
	void resolveJoints(SimBiConState *state);
	void setStance(int newStance);
	SimBiConState *getState(int idx);
	int getStateCount();
	void addState(SimBiConState *state);
	void clearStates();
	void computeTorques(vector<ContactPoint*>& cfs) override;
	void evaluateJointTargets();
	void computePDTorques(vector<ContactPoint*>& cfs);
	bool performPostTasks(double dt, vector<ContactPoint*>& cfs) override;
	virtual int advanceInTime(double dt, vector<ContactPoint*>& cfs);
	b2Vec2 getStanceFootPos();
	b2Vec2 getSwingFootPos();
	int getFSMState();
	void updateDAndV();
	void computeD0(double phi, b2Vec2& d0);
	void computeV0(double phi, b2Vec2& v0);
	static void computeDorV(double phi, Trajectory1d  *trajX, int stance, b2Vec2& result);
	
	ArticulatedRigidBody *lFoot_, *rFoot_, *root_;
	ArticulatedRigidBody *stanceFoot_, *swingFoot_;
	int lHipIndex_, rHipIndex_;
	vector<SimBiConState*> states_;
	ControlParams *rootControlParams_;
	double stanceHipDamping_, stanceHipMaxVelocity_;
	double rootPredictiveTorqueScale_, aRootD_;

	int stance_;
	int stanceHipIndex_, swingHipIndex_, FSMStateIndex_;

	b2Vec2 comVelocity_, comPosition_, d_, v_, doubleStanceCOMError_;
	double phi_;
	bool bodyTouchedTheGround_;
	int startingState_, startingStance_;
};

class VirtualModelController : public Controller {
public:
	VirtualModelController(Character* b, const string& name);
	void computeTorques(vector<ContactPoint*>& cfs) override;
	void computeJointTorquesEquivalentToForce(Joint *start, const b2Vec2 pLocal,
		const b2Vec2 fGlobal, Joint *end);
};


class BehaviourController {
public:
	BehaviourController(Character *b, IKVMCController *llc,
		WorldOracle *w = nullptr);
	virtual ~BehaviourController();	
	void setUpperBodyPose(double leanSagittal);
	void setKneeBend(double v, bool swingAlso = false);
	void setVelocities(double velDS);	
	void adjustStepHeight();
	void setElbowAngles(double leftElbowAngle, double rightElbowAngle);
	void setShoulderAngles(double leftSwing, double rightSwing);
	void requestStepTime(double stepTime);
	void requestStepHeight(double stepHeight);
	void requestVelocities(double velDS);
	void requestUpperBodyPose(double leanS);
	void requestKneeBend(double kb);
	void setDesiredSwingFootLocation();
	void initializeDefaultParameters();
	void simStepPlan(double dt);
	void conTransitionPlan();
	b2Vec2 computeSwingFootLocationEstimate(const b2Vec2 comPos, double  phase);
	double getPanicLevel();

	Character *bip_;
	IKVMCController *lowLCon_;
	WorldOracle *wo_;//own
	b2Vec2 swingFootStartPos_;
	double legLength_, ankleBaseHeight_;
	double ubSagittalLean_, velDSagittal_, kneeBend_;
	double stepTime_, stepHeight_;
};

class IKVMCController : public SimBiController {
public:
	IKVMCController(Character *b, const string& Name);
	virtual ~IKVMCController();
	void setBehaviour(BehaviourController *behaviour_disown);
	b2Vec2 computeIPStepLocation();
	void computeIKSwingLegTargets(double dt);
	void DebugComputeIKSwingLegTargets(const b2Vec2 Point);
	void computeTorques(vector<ContactPoint*>& cfs) override;
	b2Vec2 getSwingFootTargetLocation(double t, const b2Vec2 com);
	void computeGravityCompensationTorques();
	void updateSwingAndStanceReferences();
	void computeIKQandW(int parentJIndex, int childJIndex, const b2Vec2 parentAxis,
		double parentNormal, double childNormal, const b2Vec2 childEndEffector,
		const b2Vec2 wP, bool computeAngVelocities, const b2Vec2 futureWP, double dt);
	void bubbleUpTorques();
	void computeLegTorques(int ankleIndex, int kneeIndex, int hipIndex, vector<ContactPoint*>&  cfs);
	void COMJT(vector<ContactPoint*>& cfs);
	b2Vec2 computeVirtualForce();
	void preprocessAnkleVTorque(int ankleJointIndex, vector<ContactPoint*>& cfs, double& ankleVTorque);
	void performPreTasks(double dt, vector<ContactPoint*>& cfs) override;
	bool performPostTasks(double dt, vector<ContactPoint*>&  cfs) override;
	void getForceInfoOn(ArticulatedRigidBody* rb, vector<ContactPoint*>&  cfs, bool& heelForce, bool& toeForce);


	const int swingLegPlaneOfRotation = 1;
	int swingKneeIndex_;
	Joint *swingKnee_, *swingHip_;
	int lKneeIndex_, rKneeIndex_, lAnkleIndex_, rAnkleIndex_;
	int stanceAnkleIndex_, stanceKneeIndex_, swingAnkleIndex_;
	VirtualModelController *vmc_;
	double ffRootTorque_;
	BehaviourController *behaviour_;//own
	double velDSagittal_;
	Trajectory1d *swingFootTrajectorySagittal_, *swingFootHeightTrajectory_;
	bool doubleStanceMode_;
	double comOffsetSagittal_, panicHeight_, unplannedForHeight_;
};
