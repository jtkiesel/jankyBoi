#include "MotionSegment.hpp"

#include "MotionState.hpp"
#include "util.hpp"

#include <cmath>

namespace bns {

MotionSegment::MotionSegment(MotionState start, MotionState end) : mStart(start), mEnd(end) {}

bool MotionSegment::isValid() const {
	if (!epsilonEquals(mStart.acc(), mEnd.acc(), kEpsilon)) {
		// Acceleration is not constant within the segment.
		return false;
	}
	if (signum(mStart.vel()) * signum(mEnd.vel()) < 0.0 && !epsilonEquals(mStart.vel(), 0.0, kEpsilon)
			&& !epsilonEquals(mEnd.vel(), 0.0, kEpsilon)) {
		// Velocity direction reverses within the segment.
		return false;
	}
	if (!mStart.extrapolate(mEnd.t()).equals(mEnd)) {
		// A single segment is not consistent.
		if (mStart.t() == mEnd.t() && std::isinf(mStart.acc())) {
			// One allowed exception: If acc is infinite and dt is zero.
			return true;
		}
		return false;
	}
	return true;
}

bool MotionSegment::containsTime(double t) const {
	return t >= mStart.t() && t <= mEnd.t();
}

bool MotionSegment::containsPos(double pos) const {
	return (pos >= mStart.pos() && pos <= mEnd.pos()) || (pos <= mStart.pos() && pos >= mEnd.pos());
}

MotionState MotionSegment::start() const {
	return mStart;
}

void MotionSegment::setStart(MotionState start) {
	mStart = start;
}

MotionState MotionSegment::end() const {
	return mEnd;
}

void MotionSegment::setEnd(MotionState end) {
	mEnd = end;
}

}  // namespace bns
