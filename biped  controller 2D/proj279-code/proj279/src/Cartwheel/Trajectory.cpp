#include "Cartwheel/Trajectory.h"
#include <iostream>
using std::cout;


bool Trajectory1d::fancy_ = false;

int Trajectory1d::getFirstLargerIndex(double t){
	int size = Items_.size();
	if (size == 0)  return 0;
	if (t < Items_[(lastIndex_ + size - 1) % size]->t)
		lastIndex_ = 0;
	for (int i = 0; i < size; ++i) {
		int index = (i + lastIndex_) % size;
		if (t < Items_[index]->t) {
			lastIndex_ = index;
			return index;
		};
	};
	return size;
}

double Trajectory1d::getKnotValue(int i){ return Items_[i]->value; }

double Trajectory1d::getKnotPosition(int i) { return Items_[i]->t; }

void Trajectory1d::setKnotValue(int i, double  val) { 
	if (i < 0 || i > Items_.size())
		cout << "Trajectory1d access invalid vector member index.\n";
	else
		Items_[i]->value = val;
}

void Trajectory1d::setKnotPosition(int i, double  pos){
	if (i < 0 || i > Items_.size())
		cout << "Trajectory1d access invalid vector member index.\n";
	else {
		if ((i - 1 >= 0) && (Items_[i - 1]->t >= pos))  return;
		if (((i + 1) < Items_.size()) && (Items_[i + 1]->t <= pos)) return;
		Items_[i]->t = pos;
	}
}

double Trajectory1d::getMinPosition(){
	if (Items_.size() == 0) return 1e9;
	else return Items_.front()->t;
}
double Trajectory1d::getMaxPosition(){
	if (Items_.size() == 0) return -1e9;
	else return Items_.back()->t;
}
int Trajectory1d::getKnotCount(){
	return Items_.size();
}
void Trajectory1d::addKnot(double t, double val){
	int index = getFirstLargerIndex(t);
	Trajectory1dItem *item = new Trajectory1dItem(t, val);
	Items_.insert(Items_.begin() + index, item);
}
void Trajectory1d::removeKnot(int i){
	delete Items_[i];
	Items_.erase(Items_.begin() + i);
}
void Trajectory1d::clear(){
	for (int i = 0; i < Items_.size(); ++i)
		delete Items_[i];
	Items_.clear();
}
void Trajectory1d::copy(Trajectory1d *other){
	clear();
	for (int i = 0; i < other->Items_.size(); ++i) {
		Items_.push_back(
			new Trajectory1dItem(other->Items_[i]->t, other->Items_[i]->value));
	}
}
double Trajectory1d::evaluate_linear(double t){
	int size = Items_.size();
	if (size == 0)  return 0;
	if (t <= Items_[0]->t)  return Items_[0]->value;
	if (t >= Items_[size - 1]->t) 	return Items_[size - 1]->value;
	int index = getFirstLargerIndex(t);
	//now linearly interpolate between index-1 && index
	t = (t - Items_[index - 1]->t) / (Items_[index]->t - Items_[index - 1]->t);
	return Items_[index - 1]->value * (1 - t) + Items_[index]->value * t;
}
double Trajectory1d::evaluate_catmull_rom(double t) {
	double t0, t1, t2, t3, p0, p1, p2, p3, d1, d2, m1, m2;
	int size = Items_.size();
	if (size == 0)  return(0);
	if (t <= Items_[0]->t)  return(Items_[0]->value);
	if (t >= Items_[size - 1]->t) return(Items_[size - 1]->value);
	int index = getFirstLargerIndex(t);

	//now that we found the interval, get a value that indicates how far we are along it
	t = (t - Items_[index - 1]->t) / (Items_[index]->t - Items_[index - 1]->t);

	//approximate the derivatives at the two }s
	if (index - 2 < 0)
		p0 = Items_[index - 1]->value;
	else
		p0 = Items_[index - 2]->value;
	p1 = Items_[index - 1]->value;
	p2 = Items_[index]->value;
	if (index + 1 >= size)
		p3 = Items_[index]->value;
	else
		p3 = Items_[index + 1]->value;

	if (index - 2 < 0)
		t0 = Items_[index - 1]->t;
	else
		t0 = Items_[index - 2]->t;
	t1 = Items_[index - 1]->t;
	t2 = Items_[index]->t;
	if (index + 1 >= size) 
		t3 = Items_[index]->t;
	else
		t3 = Items_[index + 1]->t;

	d1 = (t2 - t0);
	d2 = (t3 - t1);

	if ((d1 > -TINY) && (d1  <  0))  d1 = -TINY;
	if ((d1 <  TINY) && (d1 >= 0))  d1 = TINY;
	if ((d2 > -TINY) && (d2  <  0))  d2 = -TINY;
	if ((d2 <  TINY) && (d2 >= 0))  d2 = TINY;

	if (fancy_)
	{
		m1 = (p2 - p0) * (1 - (t1 - t0) / d1);
		m2 = (p3 - p1) * (1 - (t3 - t2) / d2);
	}
	else
	{
		m1 = (p2 - p0)*0.5;
		m2 = (p3 - p1)*0.5;
	};

	t2 = t*t;
	t3 = t2*t;

	//&& now perform the interpolation using the four hermite basis functions from wikipedia
	return p1*(2 * t3 - 3 * t2 + 1) + m1*(t3 - 2 * t2 + t) + p2*(-2 * t3 + 3 * t2) + m2 * (t3 - t2);
}
void Trajectory1d::simplify_catmull_rom(double maxError, int nbSamples){
	if (getKnotCount() < 3)  return;

	double startTime = Items_.front()->t;
	double endTime = Items_.back()->t;

	Trajectory1d result = Trajectory1d();
	result.addKnot(startTime, Items_.front()->value);
	result.addKnot(endTime, Items_.back()->value);

	while (true) {
		double currError = 0;
		double currErrorTime = -1e9;

		for (int i = 0; i < nbSamples; ++i)	{
			double interp = i / (nbSamples - 1.0);
			double time = startTime * (1 - interp) + endTime * interp;
			double error = abs(result.evaluate_catmull_rom(time) - evaluate_catmull_rom(time));
			if (error > currError) {
				currError = error;
				currErrorTime = time;
			}
		};
		if (currError <= maxError) break;
		result.addKnot(currErrorTime, evaluate_catmull_rom(currErrorTime));
	};
	copy(&result);
	result.clear();  
}