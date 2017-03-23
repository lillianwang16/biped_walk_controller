#pragma once
#include <vector>
using std::vector;

class Trajectory1d {
public:
	const double TINY_NUMBER = 0.000000001;
	const double TINY = TINY_NUMBER;
	struct Trajectory1dItem {
		double t, value;
		Trajectory1dItem(double tt, double vv) : t(tt), value(vv) {}
	};

	Trajectory1d() : lastIndex_(0), Items_(vector<Trajectory1dItem*>()) {}
	virtual ~Trajectory1d() { clear(); }

	int getFirstLargerIndex(double t);
	double getKnotValue(int i);
	double getKnotPosition(int i);
	void setKnotValue(int i, double  val);
	void setKnotPosition(int i, double  pos);
	double getMinPosition();
	double getMaxPosition();
	int getKnotCount();
	void addKnot(double t, double val);
	void removeKnot(int i);
	void clear();
	void copy(Trajectory1d *other);
	double evaluate_linear(double t);
	double evaluate_catmull_rom(double t);
	void simplify_catmull_rom(double maxError, int nbSamples = 100);

	int lastIndex_;
	vector<Trajectory1dItem*> Items_;
	static bool fancy_;
};
