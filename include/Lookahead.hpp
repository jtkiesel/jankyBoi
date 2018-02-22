#ifndef LOOKAHEAD_HPP_
#define LOOKAHEAD_HPP_

namespace bns {

/**
 * A utility class for interpolating lookahead distance based on current
 * velocity.
 */
class Lookahead {
public:
	Lookahead(double minDist, double maxDist, double minVel, double maxVel);
	double lookaheadByVel(double vel) const;
private:
	double mMinDist;
	double mMaxDist;
	double mMinVel;
	double mMaxVel;
	double mDeltaDist;
	double mDeltaVel;
};

}

#endif  // LOOKAHEAD_HPP_
