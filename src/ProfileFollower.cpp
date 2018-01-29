#include "ProfileFollower.hpp"

#include "MotionProfileConstraints.hpp"
#include "MotionProfileGoal.hpp"
#include "MotionState.hpp"
#include "util.hpp"

#include <algorithm>
#include <cmath>

namespace bns {

ProfileFollower::ProfileFollower(double kp, double ki, double kv, double kffv, double kffa) {
	resetProfile();
	setGains(kp, ki, kv, kffv, kffa);
}

void ProfileFollower::setGains(double kp, double ki, double kv, double kffv, double kffa) {
	mKp = kp;
	mKi = ki;
	mKv = kv;
	mKffv = kffv;
	mKffa = kffa;
}

void ProfileFollower::resetProfile() {
	mTotalError = 0.0;
	mInitialState = MotionState::kInvalidState;
	mLatestActualState = MotionState::kInvalidState;
	mLatestPosError = kNaN;
	mLatestVelError = kNaN;
	mSetpointGenerator.reset();
	mGoal = MotionProfileGoal();
	mConstraints = MotionProfileConstraints();
	resetSetpoint();
}

void ProfileFollower::setGoalAndConstraints(MotionProfileGoal goal,
		MotionProfileConstraints constraints) {
	if (!mGoal.equals(goal) && mLatestSetpoint.motionState.isValid()) {
		// Clear the final state bit since the goal has been changed.
		mLatestSetpoint.finalSetpoint = false;
	}
	mGoal = goal;
	mConstraints = constraints;
}

void ProfileFollower::setGoal(MotionProfileGoal goal) {
	setGoalAndConstraints(goal, mConstraints);
}

MotionProfileGoal ProfileFollower::goal() const {
	return mGoal;
}

void ProfileFollower::setConstraints(MotionProfileConstraints constraints) {
	mConstraints = constraints;
}

MotionState ProfileFollower::setpoint() const {
	return mLatestSetpoint.motionState;
}

void ProfileFollower::resetSetpoint() {
	mLatestSetpoint = SetpointGenerator::Setpoint();
}

void ProfileFollower::resetIntegral() {
	mTotalError = 0.0;
}

double ProfileFollower::update(MotionState latestState, double t) {
	mLatestActualState = latestState;
	MotionState prevState = latestState;
	if (mLatestSetpoint.motionState.isValid()) {
		prevState = mLatestSetpoint.motionState;
	} else {
		mInitialState = prevState;
	}
	const double dt = std::max(0.0, t - prevState.t());
	mLatestSetpoint = mSetpointGenerator.setpoint(mConstraints, mGoal, prevState, t);

	// Update error.
	mLatestPosError = mLatestSetpoint.motionState.pos() - latestState.pos();
	mLatestVelError = mLatestSetpoint.motionState.vel() - latestState.vel();

	// Calculate the feedforward and proportional terms.
	double output = mKp * mLatestPosError + mKv * mLatestVelError + mKffv * mLatestSetpoint.motionState.vel()
			+ (std::isnan(mLatestSetpoint.motionState.acc() ? 0.0 : mKffa * mLatestSetpoint.motionState.acc()));
	if (output >= mMinOutput && output <= mMaxOutput) {
		// Update integral.
		mTotalError += mLatestPosError * dt;
		output += mKi * mTotalError;
	} else {
		// Reset integral windup.
		mTotalError = 0.0;
	}
	// Clamp to limits.
	return std::max(mMinOutput, std::min(output, mMaxOutput));
}

void ProfileFollower::setMinOutput(double minOutput) {
	mMinOutput = minOutput;
}

void ProfileFollower::setMaxOutput(double maxOutput) {
	mMaxOutput = maxOutput;
}

double ProfileFollower::posError() const {
	return mLatestPosError;
}

double ProfileFollower::velError() const {
	return mLatestVelError;
}

bool ProfileFollower::isFinishedProfile() const {
	return mLatestSetpoint.finalSetpoint;
}

bool ProfileFollower::onTarget() const {
	if (!mLatestSetpoint.motionState.isValid()) {
		return false;
	}
	// For the options that don't achieve the goal velocity exactly, also count
	// any instance where we have passed the finish line.
	const double goalToStart = mGoal.pos() - mInitialState.pos();
	const double goalToActual = mGoal.pos() - mLatestActualState.pos();
	const bool passedGoalState = signum(goalToStart) * signum(goalToActual) < 0.0;
	return mGoal.atGoalState(mLatestActualState)
			|| (mGoal.completionBahavior() != MotionProfileGoal::CompletionBehavior::OVERSHOOT && passedGoalState);
}

}  // namespace bns
