#pragma once
#include "Box2D.h"
#include "Cartwheel/Character.h"


class TwoLinkIK {
public:
	static b2Vec2 debug_point_;
	static b2Vec2 solve(const b2Vec2 p1, const b2Vec2 p2, double n, double r1, double r2);
	static void getIKOrientations(const b2Vec2 p1, const b2Vec2 p2, double  n, const b2Vec2 vParent, double nParent, const b2Vec2 vChild, double &aP, double &aC);
	static double getParentOrientation(const b2Vec2 vGlobal, double nGlobal, const b2Vec2 vLocal, double nLocal);
	static double getChildRotationAngle(const b2Vec2 vParent, const b2Vec2 vChild, double n);
};
