#ifndef PROFILEFOLLOWER_HPP_
#define PROFILEFOLLOWER_HPP_

#include "MotionProfileConstraints.hpp"
#include "MotionProfileGoal.hpp"
#include "MotionState.hpp"
#include "SetpointGenerator.hpp"
#include "util.hpp"

namespace bns {

/**
 * A controller for tracking a profile generated to attain a MotionProfileGoal.
 * The controller uses feedforward on acceleration, velocity, and position;
 * proportional feedback on velocity and position; and integral feedback on
 * position.
 */
class ProfileFollower {
public:
	/**
	 * Create a new ProfileFollower.
	 *
	 * @param kp    Proportional gain on position error.
	 * @param ki    Integral gain on position error.
	 * @param kv    Proportional gain on velocity error (or derivative gain on
	 * position error).
	 * @param kffv  Feedforward gain on velocity. Should be 1.0 if the units of
	 * the profile match the units of the output.
	 * @param kffa  Feedforward gain on acceleration.
	 */
	ProfileFollower(double kp, double ki, double kv, double kffv, double kffa);
	void setGains(double kp, double ki, double kv, double kffv, double kffa);
	/**
	 * Completely clear all state related to the current profile (min and max
	 * outputs are maintained).
	 */
	void resetProfile();
	/**
	 * Specify a goal and constraints for achieving the goal.
	 */
	void setGoalAndConstraints(MotionProfileGoal goal, MotionProfileConstraints constraints);
	void setGoal(MotionProfileGoal goal);
	/**
	 * @return Current goal.
	 */
	MotionProfileGoal goal() const;
	void setConstraints(MotionProfileConstraints constraints);
	MotionState setpoint() const;
	/**
	 * Reset just the setpoint. This means that the latestState provided to
	 * update() will be used rather than feeding forward the previous setpoint
	 * the next time update() is called. This almost always forces a
	 * MotionProfile update, and may be warranted if tracking error gets very
	 * large.
	 */
	void resetSetpoint();
	void resetIntegral();
	/**
	 * Update the setpoint and apply the control gains to generate a control
	 * output.
	 *
	 * @param latestState  Latest actual state, used only for feedback purposes
	 * (unless this is the first iteration or reset()/resetSetpoint() was just
	 * called, in which case this is the new start state for the profile).
	 * @param t            Timestamp for which the setpoint is desired.
	 * @return Output that reflects the control output to apply to achieve the
	 * new setpoint.
	 */
	double update(MotionState latestState, double t);
	void setMinOutput(double minOutput);
	void setMaxOutput(double maxOutput);
	double posError() const;
	double velError() const;
	/**
	 * We are finished the profile when the final setpoint has been generated.
	 * Note that this does not check whether we are anywhere close to the final
	 * setpoint, however.
	 *
	 * @return <code>true</code> if the final setpoint has been generated for
	 * the current goal.
	 */
	bool isFinishedProfile() const;
	/**
	 * We are on target if our actual state achieves the goal (where the
	 * definition of achievement depends on the goal's completion behavior).
	 *
	 * @return True if we have actually achieved the current goal.
	 */
	bool onTarget() const;
protected:
	double mKp;
	double mKi;
	double mKv;
	double mKffv;
	double mKffa;
	double mMinOutput = -kInfinity;
	double mMaxOutput = kInfinity;
	MotionState mLatestActualState;
	MotionState mInitialState;
	double mLatestPosError;
	double mLatestVelError;
	double mTotalError;
	MotionProfileGoal mGoal;
	MotionProfileConstraints mConstraints;
	SetpointGenerator mSetpointGenerator;
	SetpointGenerator::Setpoint mLatestSetpoint;
};

}  // namespace bns

#endif  // PROFILEFOLLOWER_HPP_
