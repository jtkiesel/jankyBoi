#ifndef TRAPEZOIDALMOTIONPROFILE_HPP_
#define TRAPEZOIDALMOTIONPROFILE_HPP_

#include "MotionProfile.hpp"

namespace bns {

class TrapezoidalMotionProfile : public MotionProfile {
public:
	TrapezoidalMotionProfile(double vLim, double aLim, double xi, double xf, double vi = 0, double vf = 0);
	Snapshot getSnapshot(double x, unsigned long t);
	bool isDone(unsigned long t) const;
private:
	const double vLim;
	const double aLim;
	const double xi;
	const double xf;
	const double vi;
	const double vf;
	double x1;
	unsigned long t1;
	unsigned long tf;
};

};  // namespace bns

#endif  // TRAPEZOIDALMOTIONPROFILE_HPP_
