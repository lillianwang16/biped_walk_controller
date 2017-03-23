#pragma once
#include "Cartwheel/Trajectory.h"
#include "Cartwheel/Character.h"
#include "Cartwheel/Controller.h"
#include <vector>

using namespace std;

class EditableWalking : public IKVMCController {
public:
	EditableWalking(Character* cha);
	~EditableWalking() {}
	void addControlParams2(const string& jointname, double kp, double kd, double tauMax = 1000.0, double scale = 1.0);
	void addTrajectory(SimBiConState *state, const string jointname, const vector<double>& BaseTrajectoryArray, bool relToCharFrame = false, int reverseOnStance = DONT_REVERSE );
};