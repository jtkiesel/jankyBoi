#include "TrapezoidalMotionProfile.h"

#include <math.h>
#include <stdlib.h>

#include "bnsMath.h"

TrapezoidalMotionProfile *newTrapezoidalMotionProfile(double maxVel, double maxAccel, double xi, double xf, double vi = 0, double vf = 0) {
    const double dx = xf - xi;
    // Total displacement minus minimum displacement required for vi -> vf.
	const double dxMp = dx - fabs(pow(vf, 2) - pow(vi, 2)) / (2 * maxAccel);

	if (dxMp < 0) {
		return NULL;  // Not enough displacement available for vi -> vf.
	}
	// Time required for vi -> vf.
	const unsigned long dtTransition = fabs(vi - vf) / maxAccel;
	unsigned long t1, t2, t3;
	if (vi < vf) {
		t1 = dtTransition;
		t3 = 0;
	} else {
		t1 = 0;
		t3 = dtTransition;
	}
	double baseVel = max(vi, vf);  // Base velocity for the triangle/trapezoid.

	// Displacement at max velocity (negative if max velocity can't be reached).
	double dxMaxVel = dxMp - (pow(maxVel, 2) - pow(baseVel, 2)) / (2 * maxAccel);

	double topVel, dtTopVel;
	if (dxMaxVel < 0) {  // Triangular profile.
		topVel = sqrt(pow(baseVel, 2) + 2 * maxAccel * dxMp);
		dtTopVel = 0;
	} else {  // Trapezoidal profile.
		topVel = maxVel;
		dtTopVel = dxMaxVel / maxVel;
	}
	double dtAccel = dx / (baseVel + topVel);  // Time required for baseVel -> topVel.
	t1 += dtAccel;
    const double x1 = (topVel + vi) * t1 / 2;
	t2 = t1 + dtTopVel;
    const double x2 = x1 + topVel * dtTopVel;
	t3 += t2 + dtAccel;

    TrapezoidalMotionProfile *self = (TrapezoidalMotionProfile *)malloc(sizeof(TrapezoidalMotionProfile));
    if (!self) {
        return NULL;
    }
    *self = {.maxVel = maxVel, .maxAccel = maxAccel, .xi = xi, .xf = xf, .vi = vi, .vf = vf, .t1 = t1, .x1 = x1, .t2 = t2, .x2 = x2, .t3 = t3};

    return self;
}

void freeTrapezoidalMotionProfile(TrapezoidalMotionProfile *self) {
    if (self) {
        free(self);
    }
}

TrapezoidalMotionPoint getMotionPoint(TrapezoidalMotionProfile *self, unsigned long t) {
    if (!self) {
    	return {};
    }
    double x, v, a;
	if (t < self->t1) {  // Segment 1, acceleration.
		x = self->xi + self->vi * t + self->maxAccel * pow(t, 2) / 2;
        v = self->vi + self->maxAccel * t;
        a = self->maxAccel;
	} else if (t < self->t2) {  // Segment 2, constant velocity.
        x = self->x1 + self->maxVel * t;
        v = self->maxVel;
        a = 0;
    } else {  // Segment 3, deceleration.
        t = self->t3 - t;  // Time from end.
        x = self->xf - self->vf * t - self->maxAccel * pow(t, 2) / 2;
        v = self->vf + self->maxAccel * t;
        a = -self->maxAccel;
    }
    return {.x = x, .v = v, .a = a};
}
