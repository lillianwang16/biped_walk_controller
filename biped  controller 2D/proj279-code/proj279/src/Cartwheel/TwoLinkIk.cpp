#include "Cartwheel/TwoLinkIk.h"
#include "math.h"

b2Vec2 TwoLinkIK::debug_point_ = b2Vec2(0,0);

b2Vec2 TwoLinkIK::solve(const b2Vec2 p1, const b2Vec2 p2, double n, double r1, double r2) {
	double r, a, tmp, h;
	b2Vec2 d1, d2, p;
	r = VectorDelta(p1, p2).Length();
	if (r > (r1 + r2) * 0.993) r = (r1 + r2) * 0.993;
	//this is the length of the vector starting at p1 and going to the midpoint between p1 and p2
	a = (r1 * r1 - r2 * r2 + r * r) / (2 * r);
	tmp = r1*r1 - a*a;
	if (tmp < 0) tmp = 0;
	//and this is the distance from the midpoint of p1-p2 to the intersection point
	h = sqrt(tmp);
	//now we need to get the two directions needed to reconstruct the intersection point
	d1 = VectorDelta(p1, p2);
	d1.Normalize();
	d2 = b2Cross(d1, n);
	d2.Normalize();
	//and now get the intersection point
	p.x = p1.x + d1.x * a + d2.x * (-h);
	p.y = p1.y + d1.y * a + d2.y * (-h);
	return p;
}

double safeACOS(double val) {
	if (val < -1)
		return 3.141592653;
	if (val > 1)
		return 0;
	return acos(val);
}

double angleWith(const b2Vec2 v1, const b2Vec2 v2) {
	return safeACOS(b2Dot(v1, v2) / (v1.Length() * v2.Length()));
}

double TwoLinkIK::getChildRotationAngle(const b2Vec2 vParent, const b2Vec2 vChild, double n){
	//compute the angle between the vectors (p1, p) and (p, p2), and that's our result
	double angle = angleWith(vParent, vChild);
	if (b2Cross(vParent, vChild) * n < 0)
	    angle = -angle;
	return angle;
}

double TwoLinkIK::getParentOrientation(const b2Vec2 vGlobal, double nGlobal, const b2Vec2 vLocal, double nLocal) {
	double tmp = b2Cross(vLocal, vGlobal);
	double axis = tmp / abs(tmp);
	double ang = angleWith(vLocal, vGlobal);
	return ang * axis;
}

void TwoLinkIK::getIKOrientations(const b2Vec2 p1, const b2Vec2 p2, double  n, const b2Vec2 vParent, double nParent, const b2Vec2 vChild, double &aP, double &aC) {
	b2Vec2   solvedJointPosW, vParentG, vChildG;
	double   childAngle;
	double nG = n;

	solvedJointPosW = solve(p1, p2, nG, vParent.Length(), vChild.Length());

	debug_point_ = solvedJointPosW;
	vParentG = VectorDelta(p1, solvedJointPosW);
	vChildG = VectorDelta(solvedJointPosW, p2);

	aP = getParentOrientation(vParentG, nG, vParent, nParent);

	childAngle = getChildRotationAngle(vParentG, vChildG, nG);
	aC = childAngle * nParent;
}
