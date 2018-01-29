#include "PathFollower.hpp"

#include "AdaptivePurePursuitController.hpp"
#include "MotionProfileConstraints.hpp"
#include "MotionProfileGoal.hpp"
#include "MotionState.hpp"
#include "Path.hpp"
#include "ProfileFollower.hpp"
#include "RigidTransform2d.hpp"
#include "Twist2d.hpp"

#include <algorithm>
#include <cmath>

namespace bns {

PathFollower::PathFollower(Path path, bool reversed, Parameters parameters) :
		mSteeringController(AdaptivePurePursuitController(path, reversed, parameters.kLookahead)),
		mLastSteeringDelta(Twist2d::identity()), mVelocityController(ProfileFollower(parameters.kProfileKp,
		parameters.kProfileKi, parameters.kProfileKv, parameters.kProfileKffv, parameters.kProfileKffa)),
		mProfileMaxVel(parameters.kProfileMaxVel), mProfileMaxAcc(parameters.kProfileMaxAcc),
		kGoalPosTolerance(parameters.kGoalPosTolerance), kGoalVelTolerance(parameters.kGoalVelTolerance),
		kInertiaGain(parameters.kInertiaGain), kStopSteeringDistance(parameters.kStopSteeringDistance) {
	mVelocityController.setConstraints(MotionProfileConstraints(parameters.kProfileMaxVel, parameters.kProfileMaxAcc));
};

Twist2d PathFollower::update(double t, RigidTransform2d pose, double displacement, double velocity) {
	if (!mSteeringController.isFinished()) {
		const AdaptivePurePursuitController::Command steeringCommand = mSteeringController.update(pose);
		mDebugOutput.lookaheadPointX = steeringCommand.lookaheadPoint.x();
		mDebugOutput.lookaheadPointY = steeringCommand.lookaheadPoint.y();
		mDebugOutput.lookaheadPointVel = steeringCommand.endVel;
		mDebugOutput.steeringCommandDx = steeringCommand.delta.dx();
		mDebugOutput.steeringCommandDy = steeringCommand.delta.dy();
		mDebugOutput.steeringCommandDtheta = steeringCommand.delta.dtheta();
		mCrossTrackError = steeringCommand.crossTrackError;
		mVelocityController.setGoalAndConstraints(
				MotionProfileGoal(displacement + steeringCommand.delta.dx(), std::abs(steeringCommand.endVel),
				MotionProfileGoal::CompletionBehavior::VIOLATE_MAX_ACCEL, kGoalPosTolerance, kGoalVelTolerance),
				MotionProfileConstraints(std::min(mProfileMaxVel, steeringCommand.maxVel), mProfileMaxAcc));
		if (steeringCommand.remainingPathLength < kStopSteeringDistance) {
			mDoneSteering = true;
		}
	}
	const double velocityCommand = mVelocityController.update(MotionState(t, displacement, velocity, 0.0), t);
	mAlongTrackError = mVelocityController.posError();
	double dtheta = mLastSteeringDelta.dtheta();
	const double dx = mLastSteeringDelta.dx();
	const double curvature = dtheta / dx;
	if (!std::isnan(curvature) && std::abs(curvature) < kReallyBigNumber) {
		// Regenerate angular velocity command from adjusted curvature.
		const double absVelSetpoint = std::abs(mVelocityController.setpoint().vel());
		dtheta = dx * curvature * (1.0 + kInertiaGain * absVelSetpoint);
	}
	const double scale = velocityCommand / dx;
	const Twist2d twist = Twist2d(dx * scale, 0.0, dtheta * scale);

	// Fill out debug.
	mDebugOutput.t = t;
	mDebugOutput.poseX = pose.translation().x();
	mDebugOutput.poseY = pose.translation().y();
	mDebugOutput.poseTheta = pose.rotation().radians();
	mDebugOutput.linearDisplacement = displacement;
	mDebugOutput.linearVelocity = velocity;
	mDebugOutput.profileDisplacement = mVelocityController.setpoint().pos();
	mDebugOutput.profileVelocity = mVelocityController.setpoint().vel();
	mDebugOutput.velocityCommandDx = twist.dx();
	mDebugOutput.velocityCommandDy = twist.dy();
	mDebugOutput.velocityCommandDtheta = twist.dtheta();
	mDebugOutput.crossTrackError = mCrossTrackError;
	mDebugOutput.alongTrackError = mAlongTrackError;

	return twist;
}

double PathFollower::crossTrackError() const {
	return mCrossTrackError;
}

double PathFollower::alongTrackError() const {
	return mAlongTrackError;
}

PathFollower::DebugOutput PathFollower::debug() const {
	return mDebugOutput;
}

bool PathFollower::isFinished() const {
	return (mSteeringController.isFinished() && mVelocityController.isFinishedProfile()
			&& mVelocityController.onTarget()) || mOverrideFinished;
}

void PathFollower::forceFinish() {
	mOverrideFinished = true;
}

bool PathFollower::hasPassedMarker(std::string marker) const {
	return mSteeringController.hasPassedMarker(marker);
}

}  // namespace bns
