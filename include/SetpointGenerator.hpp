#ifndef SETPOINTGENERATOR_HPP_
#define SETPOINTGENERATOR_HPP_

#include "MotionProfile.hpp"
#include "MotionProfileConstraints.hpp"
#include "MotionProfileGenerator.hpp"
#include "MotionProfileGoal.hpp"
#include "MotionState.hpp"
#include "Optional.hpp"

namespace bns {

/**
 * A SetpointGenerate does just-in-time motion profile generation to supply a
 * stream of setpoints that obey the given constraints to a controller. The
 * profile is regenerated when any of the inputs change, but is cached (and
 * trimmed as we go) if the only update is to the current state.
 *
 * Note that typically for smooth control, a user will feed the last iteration's
 * setpoint as the argument to setpoint(), and should only use a measured state
 * directly on the first iteration or if a large disturbance is detected.
 */
class SetpointGenerator {
public:
	/**
	 * A Setpoint is just a MotionState and an additional flag indicating
	 * whether this is setpoint achieves the goal (useful for higher-level logic
	 * to know that it is now time to do something else).
	 */
	class Setpoint {
	public:
		MotionState motionState;
		bool finalSetpoint;
		Setpoint() : Setpoint(MotionState::kInvalidState, false) {};
		Setpoint(MotionState motionState, bool finalSetpoint) : motionState(motionState),
				finalSetpoint(finalSetpoint) {}
	};
	SetpointGenerator();
	/**
	 * Force a reset of the profile.
	 */
	void reset();
	/**
	 * Get a new Setpoint (and generate a new MotionProfile if necessary).
	 *
	 * @param constraints  Constraints to use.
	 * @param goal         Goal to use.
	 * @param prevState    Previous setpoint (or measured state of the system to
	 * do a reset).
	 * @param t            Time to generate a setpoint for.
	 * @return New Setpoint at time t.
	 */
	Setpoint setpoint(MotionProfileConstraints constraints, MotionProfileGoal goal,
			MotionState prevState, double t);
	/**
	 * Get the full profile from the latest call to setpoint(). Useful to check
	 * estimated time or distance to goal.
	 *
	 * @return Profile from latest call to setpoint().
	 */
	MotionProfile profile() const;
protected:
	MotionProfileConstraints mConstraints;
	MotionProfileGoal mGoal;
	MotionProfile mProfile;
	bool mRegenerate;
};

}  // namespace bns

#endif  // SETPOINTGENERATOR_HPP_
