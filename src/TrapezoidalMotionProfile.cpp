#include "TrapezoidalMotionProfile.hpp"

#include <math.h>
#include <stdlib.h>
#include <algorithm>

#include "bnsMath.hpp"

namespace bns {

TrapezoidalMotionProfile::TrapezoidalMotionProfile(double maxVel, double maxAcc, double xi, double xf, double vi, double vf) {
	const double dx = xf - xi;
	// Total displacement minus minimum displacement required for vi -> vf.
	const double dxMp = dx - fabs(pow(vf, 2) - pow(vi, 2)) / (2 * maxAcc);

	if (dxMp < 0) {
		return;  // Not enough displacement available for vi -> vf.
	}
	// Time required for vi -> vf.
	const unsigned long dtTransition = fabs(vi - vf) / maxAcc;
	unsigned long t1, t2, t3;
	if (vi < vf) {
		t1 = dtTransition;
		t3 = 0;
	} else {
		t1 = 0;
		t3 = dtTransition;
	}
	double baseVel = std::max(vi, vf);  // Base velocity for the triangle/trapezoid.

	// Displacement at max velocity (negative if max velocity can't be reached).
	double dxMaxVel = dxMp - (pow(maxVel, 2) - pow(baseVel, 2)) / (2 * maxAcc);

	double topVel, dtTopVel;
	if (dxMaxVel < 0) {  // Triangular profile.
		topVel = sqrt(pow(baseVel, 2) + 2 * maxAcc * dxMp);
		dtTopVel = 0;
	} else {  // Trapezoidal profile.
		topVel = maxVel;
		dtTopVel = dxMaxVel / maxVel;
	}
	double dtAcc = dx / (baseVel + topVel);  // Time required for baseVel -> topVel.
	t1 += dtAcc;
	const double x1 = (vi + topVel) * t1 / 2;
	t2 = t1 + dtTopVel;
	const double x2 = x1 + topVel * dtTopVel;
	t3 += t2 + dtAcc;

	this->maxVel = maxVel;
	this->maxAcc = maxAcc;
	this->xi = xi;
	this->xf = xf;
	this->vi = vi;
	this->vf = vf;
	this->t1 = t1;
	this->x1 = x1;
	this->t2 = t2;
	this->x2 = x2;
	this->t3 = t3;
}

MotionProfile::Snapshot TrapezoidalMotionProfile::getSnapshot(unsigned long t) const {
	Snapshot snapshot;
	if (t < t1) {  // Segment 1, acceleration.
		snapshot.x = xi + vi * t + maxAcc * pow(t, 2) / 2;
		snapshot.v = vi + maxAcc * t;
		snapshot.a = maxAcc;
	} else if (t < t2) {  // Segment 2, constant velocity.
		snapshot.x = x1 + maxVel * t;
		snapshot.v = maxVel;
		snapshot.a = 0;
	} else {  // Segment 3, deceleration.
		t = t3 - t;  // Time from end.
		snapshot.x = xf - vf * t - maxAcc * pow(t, 2) / 2;
		snapshot.v = vf + maxAcc * t;
		snapshot.a = -maxAcc;
	}
	return snapshot;
}

}  // namespace bns
