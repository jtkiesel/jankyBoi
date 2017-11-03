#ifndef TRAPEZOIDALMOTIONPROFILE_H_
#define TRAPEZOIDALMOTIONPROFILE_H_

typedef struct {
    double maxVel;
    double maxAccel;

    double xi;
    double xf;

    double vi;
    double vf;

    unsigned long t1;
    double x1;
    unsigned long t2;
    double x2;
    unsigned long t3;
} TrapezoidalMotionProfile;

typedef struct {
    double x;
    double v;
    double a;
} TrapezoidalMotionPoint;

TrapezoidalMotionProfile *newTrapezoidalMotionProfile(double maxVel, double maxAccel, double xi, double xf, double vi = 0, double vf = 0);

void freeTrapezoidalMotionProfile(TrapezoidalMotionProfile *self);

TrapezoidalMotionPoint getMotionPoint(TrapezoidalMotionProfile *self, unsigned long t);

#endif  // TRAPEZOIDALMOTIONPROFILE_H_
