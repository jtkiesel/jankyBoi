#ifndef MOTIONPROFILEGOAL_HPP_
#define MOTIONPROFILEGOAL_HPP_

#include "MotionState.hpp"

namespace bns {

/**
 * A MotionProfileGoal defines a desired position and maximum velocity (at this
 * position), along with the behavior that should be used to determine if we are
 * at the goal and what to do if it is infeasible to reach the goal within the
 * desired velocity bounds.
 */
class MotionProfileGoal {
public:
	/**
	 * A goal consists of a desired position and specified maximum velocity
	 * magnitude. But what should we do if we would reach the goal at a velocity
	 * greater than the maximum? This enum allows a user to specify a preference
	 * on behavior in this case.
	 *
	 * Example use-cases of each:
	 *
	 * OVERSHOOT - Generally used with a goal maxAbsVel of 0.0 to stop at the
	 * desired pos without violating any constraints.
	 *
	 * VIOLATE_MAX_ACCEL - If we absolutely do not want to pass the goal and are
	 * unwilling to violate the maxAbsVel (for example, there is an obstacle in
	 * front of us - slam the brakes harder than we'd like in order to avoid
	 * hitting it).
	 *
	 * VIOLATE_MAX_VEL - If the max velocity is just a general guideline and not
	 * a hard performance limit, it's better to slightly exceed it to avoid
	 * skidding wheels.
	 */
	enum class CompletionBehavior {
		// Overshoot the goal if necessary (at a velocity greater than
		// maxAbsVel) and come back. Only valid if the goal velocity is 0.0
		// (otherwise VIOLATE_MAX_ACCEL will be used).
		OVERSHOOT,
		// If we cannot slow down to the goal velocity before crossing the goal,
		// allow exceeding the max accel constraint.
		VIOLATE_MAX_ACCEL,
		// If we cannot slow down to the goal velocity before crossing the goal,
		// allow exceeding the goal velocity.
		VIOLATE_MAX_VEL
	};
	MotionProfileGoal();
	MotionProfileGoal(double pos);
	MotionProfileGoal(double pos, double maxVel);
	MotionProfileGoal(double pos, double maxVel, CompletionBehavior completionBehavior);
	MotionProfileGoal(double pos, double maxVel, CompletionBehavior completionBehavior,
			double posTolerance, double velTolerance);
	MotionProfileGoal(const MotionProfileGoal &other);
	/**
	 * @return A flipped MotionProfileGoal (where the position is negated, but
	 * all other attributes remain the same).
	 */
	MotionProfileGoal flipped() const;
	double pos() const;
	double maxVel() const;
	double posTolerance() const;
	double velTolerance() const;
	CompletionBehavior completionBahavior() const;
	bool atGoalState(MotionState state) const;
	bool atGoalPos(double pos) const;
	/**
	 * This method makes sure that the completion behavior is compatible with
	 * the max goal velocity.
	 */
	void sanityCheck();
	bool equals(MotionProfileGoal other) const;
protected:
	double mPos;
	double mMaxVel;
	CompletionBehavior mCompletionBehavior;
	double mPosTolerance;
	double mVelTolerance;
};

}

#endif  // MOTIONPROFILEGOAL_HPP_
