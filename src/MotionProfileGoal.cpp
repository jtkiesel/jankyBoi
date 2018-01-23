#include "MotionProfileGoal.hpp"

#include "util.hpp"

#include <cmath>

namespace bns {

MotionProfileGoal::MotionProfileGoal() :
 		MotionProfileGoal(0.0, 0.0, CompletionBehavior::OVERSHOOT, 1E-3, 1E-2) {}

MotionProfileGoal::MotionProfileGoal(double pos) :
 		MotionProfileGoal(pos, 0.0, CompletionBehavior::OVERSHOOT, 1E-3, 1E-2) {}

MotionProfileGoal::MotionProfileGoal(double pos, double maxVel) :
 		MotionProfileGoal(pos, maxVel, CompletionBehavior::OVERSHOOT, 1E-3, 1E-2) {}

MotionProfileGoal::MotionProfileGoal(double pos, double maxVel, CompletionBehavior completionBehavior) :
 		MotionProfileGoal(pos, maxVel, completionBehavior, 1E-3, 1E-2) {}

MotionProfileGoal::MotionProfileGoal(double pos, double maxVel, CompletionBehavior completionBehavior,
		double posTolerance, double velTolerance) : mPos(pos), mMaxVel(maxVel),
		mCompletionBehavior(completionBehavior),  mPosTolerance(posTolerance), mVelTolerance(velTolerance) {
	sanityCheck();
}

MotionProfileGoal::MotionProfileGoal(const MotionProfileGoal &other) : MotionProfileGoal(other.mPos,
		other.mMaxVel, other.mCompletionBehavior, other.mPosTolerance, other.mVelTolerance) {}

MotionProfileGoal MotionProfileGoal::flipped() const {
	return MotionProfileGoal(-mPos, mMaxVel, mCompletionBehavior, mPosTolerance, mVelTolerance);
}

double MotionProfileGoal::pos() const {
	return mPos;
}

double MotionProfileGoal::maxVel() const {
	return mMaxVel;
}

double MotionProfileGoal::posTolerance() const {
	return mPosTolerance;
}

double MotionProfileGoal::velTolerance() const {
	return mVelTolerance;
}

MotionProfileGoal::CompletionBehavior MotionProfileGoal::completionBahavior() const {
	return mCompletionBehavior;
}

bool MotionProfileGoal::atGoalState(MotionState state) const {
	return atGoalPos(state.pos()) && (std::abs(state.vel()) < (mMaxVel + mVelTolerance)
			|| mCompletionBehavior == CompletionBehavior::VIOLATE_MAX_VEL);
}

bool MotionProfileGoal::atGoalPos(double pos) const {
	return epsilonEquals(pos, mPos, mPosTolerance);
}

void MotionProfileGoal::sanityCheck() {
	if (mMaxVel > mVelTolerance && mCompletionBehavior == CompletionBehavior::OVERSHOOT) {
		mCompletionBehavior = CompletionBehavior::VIOLATE_MAX_ACCEL;
	}
}

bool MotionProfileGoal::equals(MotionProfileGoal other) const {
	return (other.mCompletionBehavior == mCompletionBehavior) && (other.mPos == mPos)
			&& (other.mMaxVel == mMaxVel) && (other.mPosTolerance == mPosTolerance)
			&& (other.mVelTolerance == other.mVelTolerance);
}

}  // namespace bns
