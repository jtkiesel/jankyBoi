#ifndef MOTIONPROFILE_HPP_
#define MOTIONPROFILE_HPP_

namespace bns {

class MotionProfile {
public:
	struct Snapshot {
		double x;
		double v;
		double a;
	} snapshot;
    virtual Snapshot computeSnapshot(double x, unsigned long t) = 0;
	virtual Snapshot getSnapshot() const = 0;
	virtual bool isDone(unsigned long t) const = 0;
	void graph(double x) const;
};

};  // namespace bns

#endif  // MOTIONPROFILE_HPP_
