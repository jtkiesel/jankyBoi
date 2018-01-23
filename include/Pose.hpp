#ifndef POSE_HPP_
#define POSE_HPP_

#include "api.hpp"

namespace bns {

class Pose {
public:
	Pose();
	Pose(double x, double y, double theta);
	double x() const;
	void setX(double x);
	double y() const;
	void setY(double y);
	double theta() const;
	void setTheta(double theta);
	void add(Pose pose);
	void add(double x, double y, double theta);
private:
	const pros::Mutex mutex;
	double mX;
	double mY;
	double mTheta;
};

}  // namespace bns

#endif  // POSE_HPP_
