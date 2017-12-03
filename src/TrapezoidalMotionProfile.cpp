#include "TrapezoidalMotionProfile.hpp"

#include <cmath>
#include <stdlib.h>
#include <algorithm>

#include "bnsMath.hpp"

namespace bns {

TrapezoidalMotionProfile::TrapezoidalMotionProfile(double vLim, double aLim, double vi, double vf) : vLim(std::abs(vLim)), aLim(std::abs(aLim)), vi(vi), vf(vf) {
	tf = 0;
}

MotionProfile::Snapshot TrapezoidalMotionProfile::getSnapshot(double error, unsigned long t) const {
	const double dir = sgn(error);  // Direction.
	// Velocity as function of time (for acceleration).
	double v = vi + dir * aLim * t;  // v(t) = v0 + a*t

	// Limit velocity.
	if (std::abs(v) > vLim) {
		v = vLim;
	}
	Snapshot snapshot;
	// Determine whether or not it is time to decelerate.  x = (vf^2 - vi^2) / (2*a)
	if (std::abs(error) < (std::pow(vf, 2) - std::pow(v, 2)) / (2 * dir * aLim)) {
		if (tf == 0) {
			tf = (v - vf) / aLim;  //
		}
		// Velocity as function of error (for deceleration).
		snapshot.v = dir * std::sqrt(std::pow(vf, 2) + 2 * aLim * error);
		snapshot.a = dir * aLim;
	}
	return snapshot;
}

bool TrapezoidalMotionProfile::isDone(unsigned long t) const {
	return t >= tf;
}

}  // namespace bns
