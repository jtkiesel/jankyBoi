#ifndef MOTIONPROFILEGENERATOR_HPP_
#define MOTIONPROFILEGENERATOR_HPP_

#include "MotionProfile.hpp"
#include "MotionProfileConstraints.hpp"
#include "MotionProfileGoal.hpp"
#include "MotionState.hpp"

namespace bns {

/**
 * A MotionProfileGenerator generates minimum-time MotionProfiles to travel from
 * a given MotionState to a given MotionProfileGoal while obeying a set of
 * MotionProfileConstraints.
 */
class MotionProfileGenerator {
public:
	/**
	* Generate a motion profile.
	*
	* @param constraints  The constraints to use.
	* @param goalState    The goal to use.
	* @param prevState    The initial state to use.
	* @return A motion profile from prevState to goalState that satisfies
	* constraints.
	*/
	static MotionProfile generateProfile(MotionProfileConstraints constraints,
			MotionProfileGoal goalState, MotionState prevState);
protected:
	static MotionProfile generateFlippedProfile(MotionProfileConstraints constraints,
			MotionProfileGoal goalState, MotionState prevState);
private:
	// Static class.
	MotionProfileGenerator();
};

}  // namespace bns

#endif  // MOTIONPROFILEGENERATOR_HPP_
