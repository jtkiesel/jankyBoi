#ifndef TRAPEZOIDALMOTIONPROFILE_HPP_
#define TRAPEZOIDALMOTIONPROFILE_HPP_

#include "MotionProfile.hpp"

namespace bns {

class TrapezoidalMotionProfile : public MotionProfile {
public:
	TrapezoidalMotionProfile(double vLim, double aLim, double vi = 0, double vf = 0);
	Snapshot getSnapshot(double error, unsigned long t) const;
	bool isDone(unsigned long t) const;
private:
	const double vLim;
	const double aLim;
	const double vi;
	const double vf;
	mutable unsigned long tf;
};

};  // namespace bns

#endif  // TRAPEZOIDALMOTIONPROFILE_HPP_
