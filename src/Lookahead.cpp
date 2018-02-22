#include "Lookahead.hpp"

#include <algorithm>
#include <cmath>

namespace bns {

Lookahead::Lookahead(double minDist, double maxDist, double minVel, double maxVel) :
		mMinDist(minDist), mMaxDist(maxDist), mMinVel(minVel), mMaxVel(maxVel),
		mDeltaDist(maxDist - minDist), mDeltaVel(maxVel - minVel) {}

double Lookahead::lookaheadByVel(double vel) const {
	double lookahead = mDeltaDist * (vel - mMinVel) / mDeltaVel + mMinDist;
	return std::isnan(lookahead) ? mMinDist : std::max(mMinDist, std::min(lookahead, mMaxDist));
}

}  // namespace bns
