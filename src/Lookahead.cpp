#include "Lookahead.hpp"

#include <algorithm>
#include <cmath>

namespace bns {

Lookahead::Lookahead(double minDist, double maxDist, double minVel, double maxVel) :
		kMinDist(minDist), kMaxDist(maxDist), kMinVel(minVel), kMaxVel(maxVel),
		kDeltaDist(maxDist - minDist), kDeltaVel(maxVel - minVel) {}

double Lookahead::lookaheadByVel(double vel) const {
	double lookahead = kDeltaDist * (vel - kMinVel) / kDeltaVel + kMinDist;
	return std::isnan(lookahead) ? kMinDist : std::max(kMinDist, std::min(lookahead, kMaxDist));
}

}  // namespace bns
