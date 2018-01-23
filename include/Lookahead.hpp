#ifndef LOOKAHEAD_HPP_
#define LOOKAHEAD_HPP_

namespace bns {

/**
 * A utility class for interpolating lookahead distance based on current speed.
 */
class Lookahead {
public:
	const double kMinDist;
	const double kMaxDist;
	const double kMinVel;
	const double kMaxVel;
	Lookahead(double minDist, double maxDist, double minVel, double maxVel);
	double lookaheadByVel(double vel) const;
protected:
	const double kDeltaDist;
	const double kDeltaVel;
};

}

#endif  // LOOKAHEAD_HPP_
