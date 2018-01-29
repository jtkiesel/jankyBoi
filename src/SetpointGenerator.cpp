#include "SetpointGenerator.hpp"

#include "MotionProfileConstraints.hpp"
#include "MotionProfileGoal.hpp"
#include "MotionState.hpp"
#include "util.hpp"

#include <algorithm>
#include <cmath>

namespace bns {

SetpointGenerator::SetpointGenerator() {
	reset();
}

void SetpointGenerator::reset() {
	mConstraints = MotionProfileConstraints();
	mGoal = MotionProfileGoal();
 	mProfile = MotionProfile();
	mRegenerate = true;
}

SetpointGenerator::Setpoint SetpointGenerator::setpoint(MotionProfileConstraints constraints, MotionProfileGoal goal,
		MotionState prevState, double t) {
	bool regenerate = mRegenerate || !mConstraints.equals(constraints) || !mGoal.equals(goal);
	if (!regenerate && !mProfile.isEmpty()) {
		Optional<MotionState> expectedState = mProfile.stateByTime(prevState.t());
		regenerate = !expectedState.isPresent() || !expectedState.get().equals(prevState);
	}
	if (regenerate) {
		// Regenerate the profile, as our current profile does not satisfy the
		// inputs.
		mConstraints = constraints;
		mGoal = goal;
		mProfile = MotionProfileGenerator::generateProfile(constraints, goal, prevState);
	}
	// In case of invalid or empty profile - just output the same state again.
	Setpoint setpoint = Setpoint(prevState, true);

	// Sample the profile at time t.
	if (!mProfile.isEmpty() && mProfile.isValid()) {
		MotionState state = mProfile.stateByTime(t).get();
		if (t > mProfile.endTime()) {
			state = mProfile.endState();
		} else if (t < mProfile.startTime()) {
			state = mProfile.startState();
		}
		// Shorten the profile and return the new setpoint.
		mProfile.trimBeforeTime(t);
		setpoint = Setpoint(prevState, mProfile.isEmpty() || mGoal.atGoalState(state));
	}
	if (setpoint.finalSetpoint) {
		// Ensure the final setpoint matches the goal exactly.
		setpoint.motionState = MotionState(setpoint.motionState.t(), mGoal.pos(),
				signum(setpoint.motionState.vel()) * std::max(mGoal.maxVel(), std::abs(setpoint.motionState.vel())),
				0.0);
	}
	return setpoint;
}

MotionProfile SetpointGenerator::profile() const {
	return mProfile;
}

}  // namespace bns
