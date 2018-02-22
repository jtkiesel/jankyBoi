#include "MotionProfileGenerator.hpp"

#include "MotionProfile.hpp"
#include "MotionProfileConstraints.hpp"
#include "MotionProfileGoal.hpp"
#include "MotionSegment.hpp"
#include "MotionState.hpp"
#include "util.hpp"

#include <cmath>
#include <limits>

namespace bns {

MotionProfileGenerator::MotionProfileGenerator() {}

MotionProfile MotionProfileGenerator::generateFlippedProfile(MotionProfileConstraints constraints,
		MotionProfileGoal goalState, MotionState prevState) {
	MotionProfile profile = generateProfile(constraints, goalState.flipped(), prevState.flipped());
	profile.flipSegments();
	return profile;
}

MotionProfile MotionProfileGenerator::generateProfile(MotionProfileConstraints constraints,
		MotionProfileGoal goalState, MotionState prevState) {
	double deltaPos = goalState.pos() - prevState.pos();
	if (deltaPos < 0.0 || (deltaPos == 0.0 && prevState.vel() < 0.0)) {
		// For simplicity, we always assume the goal requires positive movement.
		// If negative, we flip to solve, then flip the solution.
		return generateFlippedProfile(constraints, goalState, prevState);
	}
	// Invariant from this point on: deltaPos >= 0.0.
	// Clamp the start state to be valid.
	MotionState startState = MotionState(prevState.t(), prevState.pos(),
			signum(prevState.vel()) * std::min(std::abs(prevState.vel()), constraints.maxVel()),
			signum(prevState.acc()) * std::min(std::abs(prevState.acc()), constraints.maxAcc()));
	MotionProfile profile = MotionProfile();
	profile.reset(startState);
	// If our velocity is headed away from the goal, the first thing we need to
	// do is to stop.
	if (startState.vel() < 0.0 && deltaPos > 0.0) {
		const double stoppingTime = std::abs(startState.vel() / constraints.maxAcc());
		profile.appendControl(constraints.maxAcc(), stoppingTime);
		startState = profile.endState();
		deltaPos = goalState.pos() - startState.pos();
	}
	// Invariant from this point on: startState.vel() >= 0.0.
	const double minVelAtGoal2 = startState.vel2() - 2.0 * constraints.maxAcc() * deltaPos;
	const double minVelAtGoal = std::sqrt(std::abs(minVelAtGoal2));
	const double maxVelAtGoal = std::sqrt(startState.vel2() + 2.0 * constraints.maxAcc() * deltaPos);
	double goalVel = goalState.maxVel();
	double maxAcc = constraints.maxAcc();
	if (minVelAtGoal2 > 0.0 && minVelAtGoal > (goalState.maxVel() + goalState.velTolerance())) {
		// Overshoot is unavoidable with the current constraints. Look at
		// completionBehavior to see what we should do.
		if (goalState.completionBahavior() == MotionProfileGoal::CompletionBehavior::VIOLATE_MAX_VEL) {
			// Adjust the goal velocity.
			goalVel = minVelAtGoal;
		} else if (goalState.completionBahavior() == MotionProfileGoal::CompletionBehavior::VIOLATE_MAX_ACCEL) {
			if (std::abs(deltaPos) < goalState.posTolerance()) {
				// Special case: We are at the goal but moving too fast. This
				// requires 'infinite' acceleration, which will result in NaNs
				// below, so we can return the profile immediately.
				profile.appendSegment(MotionSegment(
						MotionState(profile.endTime(), profile.endPos(), profile.endState().vel(), -std::numeric_limits<double>::infinity()),
						MotionState(profile.endTime(), profile.endPos(), goalVel, -std::numeric_limits<double>::infinity())));
				profile.consolidate();
				return profile;
			}
			// Adjust the max acceleration.
			maxAcc = std::abs(goalVel * goalVel - startState.vel2()) / (2.0 * deltaPos);
		} else {
			// We are going to overshoot the goal, so the first thing we need to
			// do is come to a stop.
			const double stoppingTime = std::abs(startState.vel() / constraints.maxAcc());
			profile.appendControl(-constraints.maxAcc(), stoppingTime);
			// Now we need to travel backwards, so generate a flipped profile.
			profile.appendProfile(generateFlippedProfile(constraints, goalState, profile.endState()));
			profile.consolidate();
			return profile;
		}
	}
	goalVel = std::min(goalVel, maxVelAtGoal);
	// Invariant from this point forward: We can achieve goalVel at
	// goalState.pos() exactly using no more than +/- maxAcc.

	// What is the maximum velocity we can reach (maxVel)? This is the
	// intersection of two curves: one accelerating towards the goal from
	// profile.endState(), the other coming from the goal at maxVel (in
	// reverse). If maxVel is greater than constraints.maxVel(), we will clamp
	// and cruise.
	// Solve the following three equations to find maxVel (by substitution):
	// maxVel^2 = startVel^2 + 2*a*distAccel
	// goalVel^2 = maxVel^2 - 2*a*distDecel
	// deltaPos = distAccel + distDecel
	const double maxVel = std::min(constraints.maxVel(),
			std::sqrt((startState.vel2() + goalVel * goalVel) / 2.0 + deltaPos * maxAcc));

	// Accelerate to maxVel.
	if (maxVel > startState.vel()) {
		const double accelTime = (maxVel - startState.vel()) / maxAcc;
		profile.appendControl(maxAcc, accelTime);
		startState = profile.endState();
	}
	// Figure out how much distance will be covered during deceleration.
	const double distDecel = std::max(0.0, (startState.vel2() - goalVel * goalVel) / (2.0 * constraints.maxAcc()));
	const double distCruise = std::max(0.0, goalState.pos() - startState.pos() - distDecel);
	// Cruise at constant velocity.
	if (distCruise > 0.0) {
		const double cruiseTime = distCruise / startState.vel();
		profile.appendControl(0.0, cruiseTime);
		startState = profile.endState();
	}
	// Decelerate to goal velocity.
	if (distDecel > 0.0) {
		const double decelTime = (startState.vel() - goalVel) / maxAcc;
		profile.appendControl(-maxAcc, decelTime);
	}
	profile.consolidate();
	return profile;
}

}  // namespace bns
