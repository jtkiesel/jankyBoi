#ifndef MOTIONSEGMENT_HPP_
#define MOTIONSEGMENT_HPP_

#include "MotionState.hpp"

namespace bns {

/**
 * A MotionSegment is a movement from a start MotionState to an end MotionState with a constant acceleration.
 */
class MotionSegment {
public:
	MotionSegment(MotionState start, MotionState end);
	/**
	 * Verifies that:
	 * 1. All segments have a constant acceleration.
	 * 2. All segments have monotonic position (sign of velocity doesn't change).
	 * 3. The time, position, velocity, and acceleration of the profile are consistent.
	 */
	bool isValid() const;
	bool containsTime(double t) const;
	bool containsPos(double pos) const;
	MotionState start() const;
	void setStart(MotionState start);
	MotionState end() const;
	void setEnd(MotionState end);
protected:
	MotionState mStart;
	MotionState mEnd;
private:
	static constexpr double kEpsilon = 1E-6;
};

}  // namespace bns

#endif  // MOTIONSEGMENT_HPP_
