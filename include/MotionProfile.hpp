#ifndef MOTIONPROFILE_HPP_
#define MOTIONPROFILE_HPP_

namespace bns {

class MotionProfile {
public:
	struct Snapshot {
		double x;
		double v;
		double a;
	};
    virtual Snapshot getSnapshot(unsigned long t) const = 0;
};

};  // namespace bns

#endif  // MOTIONPROFILE_HPP_
