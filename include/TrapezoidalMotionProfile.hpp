#ifndef TRAPEZOIDALMOTIONPROFILE_HPP_
#define TRAPEZOIDALMOTIONPROFILE_HPP_

namespace bns {

class TrapezoidalMotionProfile {
public:
    typedef struct {
        double x;
        double v;
        double a;
    } Snapshot;
    TrapezoidalMotionProfile(double maxVel, double maxAcc, double xi, double xf, double vi = 0, double vf = 0);
    Snapshot getSnapshot(unsigned long t);
private:
    double maxVel;
    double maxAcc;
    double xi;
    double xf;
    double vi;
    double vf;
    unsigned long t1;
    double x1;
    unsigned long t2;
    double x2;
    unsigned long t3;
};

}  // namespace bns

#endif  // TRAPEZOIDALMOTIONPROFILE_HPP_
