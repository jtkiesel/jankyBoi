#include "TrapezoidalMotionProfile.hpp"

#include <cmath>

namespace bns {

TrapezoidalMotionProfile::TrapezoidalMotionProfile(double vLim, double aLim, double xi, double xf, double vi, double vf) : vLim(std::copysign(vLim, xf - xi)), aLim(std::copysign(aLim, xf - xi)), xi(xi), xf(xf), vi(vi), vf(vf) {
	t1 = std::abs((this->vLim - vi) / this->aLim);  // t = (vf - vi) / a
	x1 = xi + (vi + this->vLim) * t1 / 2;  // d = (vi + vf) * t / 2
	tf = 0;
}

MotionProfile::Snapshot TrapezoidalMotionProfile::getSnapshot(double x, unsigned long t) {
	Snapshot snapshot;

	if (tf == 0) {
		if (t < t1) {  // Acceleration
			snapshot.a = aLim;
			snapshot.v = vi + aLim * t;  // v = vi + a*t
			snapshot.x = xi + vi * t + aLim * std::pow(t, 2) / 2;  // x = xi + vi*t + (a*t^2)/2
		} else {  // Constant velocity.
			snapshot.a = 0;
			snapshot.v = vLim;
			snapshot.x = x1 + vLim * (t - t1);
		}
		// Decelerate.  dx = (vf^2 - vi^2) / (2*a)
		if (std::abs(xf - x) < std::abs((std::pow(vf, 2) - std::pow(snapshot.v, 2)) / (2 * aLim))) {
			tf = t + std::abs((vf - snapshot.v) / aLim);  // Time after deceleration. t = (vf - v) / a
		}
	}
	if (tf != 0) {
		if (t < tf) {
			const double t2 = tf - t;
			// Deceleration.
			snapshot.a = -aLim;
			snapshot.v = vf - aLim * t2;  // v = vf - a*(tf - t)
			snapshot.x = xf - vf * t2 - aLim * std::pow(t2, 2) / 2;
		} else {
			// Done.
			snapshot.a = 0;
			snapshot.v = vf;
			snapshot.x = xf + vf * (t - tf);
		}
	}
	return snapshot;
}

bool TrapezoidalMotionProfile::isDone(unsigned long t) const {
	return (tf != 0) && (t >= tf);
}

}  // namespace bns
