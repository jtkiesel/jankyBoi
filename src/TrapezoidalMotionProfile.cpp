#include "TrapezoidalMotionProfile.hpp"

#include <API.hpp>
#include <cmath>

namespace bns {

TrapezoidalMotionProfile::TrapezoidalMotionProfile(double vLim, double aLim, double xi, double xf, double vi, double vf)
		: mutex(mutexCreate()), vLim(std::copysign(vLim, xf - xi)), aLim(std::copysign(aLim, xf - xi)), xi(xi), xf(xf), vi(vi), vf(vf) {
	t1 = 0;  // t = (vf - vi) / a
	x1 = 0;  // d = (vi + vf) * t / 2
	tf = 0;
	snapshot = {.x = 0, .v = 0, .a = 0};
}

MotionProfile::Snapshot TrapezoidalMotionProfile::computeSnapshot(double x, unsigned long t) {
	double xSnap = 0, vSnap = 0, aSnap = 0;

	if (tf == 0) {
		if (t1 == 0) {
			// Acceleration.
			xSnap = xi + vi * t + aLim * std::pow(t, 2) / 2;  // x = xi + vi*t + (a*t^2)/2
			vSnap = vi + aLim * t;  // v = vi + a*t
			aSnap = aLim;

			if (vSnap > vLim) {
				t1 = t;
				x1 = xSnap;
			}
		}
		if (t1 != 0) {
			// Constant velocity.
			xSnap = x1 + vLim * (t - t1);  // x = xi + v * t
			vSnap = vLim;
			aSnap = 0;
		}
		// Deceleration.
		double t2 = std::abs((vf - vSnap) / aLim);
		if (std::abs(xf - x) < std::abs(vf + vSnap) * t2 / 2) {
			tf = t + t2;  // Time after deceleration. t = (vf - v) / a
		}
	}
	if (tf != 0) {
		if (t < tf) {  // Deceleration.
			const double t2 = tf - t;
			xSnap = xf - vf * t2 - aLim * std::pow(t2, 2) / 2;
			vSnap = vf + aLim * t2;  // v = vf - a*(tf - t)
			aSnap = -aLim;
		} else {  // Done.
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
