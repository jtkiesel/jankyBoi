#include "TrapezoidalMotionProfile.hpp"

#include "API.hpp"

#include <cmath>

namespace bns {

TrapezoidalMotionProfile::TrapezoidalMotionProfile(double vLim, double aLim, double xi, double xf, double vi, double vf)
		: mutex(mutexCreate()), vLim(std::copysign(vLim, xf - xi)), aLim(std::copysign(aLim, xf - xi)), xi(xi), xf(xf), vi(vi), vf(vf) {
	t1 = std::abs((this->vLim - vi) / this->aLim);  // t = (vf - vi) / a
	x1 = xi + (vi + this->vLim) * t1 / 2;  // d = (vi + vf) * t / 2
	tf = 0;
	snapshot = {.x = 0, .v = 0, .a = 0};
}

MotionProfile::Snapshot TrapezoidalMotionProfile::computeSnapshot(double x, unsigned long t) {
	double aSnap, vSnap, xSnap;

	if (tf == 0) {
		if (t < t1) {  // Acceleration
			xSnap = xi + vi * t + aLim * std::pow(t, 2) / 2;  // x = xi + vi*t + (a*t^2)/2
			vSnap = vi + aLim * t;  // v = vi + a*t
			aSnap = aLim;
		} else {  // Constant velocity.
			xSnap = x1 + vLim * (t - t1);
			vSnap = vLim;
			aSnap = 0;
		}
		// Decelerate.  dx = (vf^2 - vi^2) / (2*a)
		if (std::abs(xf - x) < std::abs((std::pow(vf, 2) - std::pow(vSnap, 2)) / (2 * aLim))) {
			tf = t + std::abs((vf - vSnap) / aLim);  // Time after deceleration. t = (vf - v) / a
		}
	}
	if (tf != 0) {
		if (t < tf) {
			const double t2 = tf - t;
			// Deceleration.
			xSnap = xf - vf * t2 - aLim * std::pow(t2, 2) / 2;
			vSnap = vf - aLim * t2;  // v = vf - a*(tf - t)
			aSnap = -aLim;
		} else {
			// Done.
			xSnap = xf + vf * (t - tf);
			vSnap = vf;
			aSnap = 0;
		}
	}
	mutexTake(mutex, 20);
	snapshot = {.x = xSnap, .v = vSnap, .a = aSnap};
	mutexGive(mutex);
	return snapshot;
}

MotionProfile::Snapshot TrapezoidalMotionProfile::getSnapshot() const {
	mutexTake(mutex, 20);
	Snapshot copy = snapshot;
	mutexGive(mutex);
	return copy;
}

bool TrapezoidalMotionProfile::isDone(unsigned long t) const {
	return (tf != 0) && (t >= tf);
}

};  // namespace bns
