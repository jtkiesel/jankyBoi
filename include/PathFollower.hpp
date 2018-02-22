#ifndef PATHFOLLOWER_HPP_
#define PATHFOLLOWER_HPP_

#include "AdaptivePurePursuitController.hpp"
#include "Lookahead.hpp"
#include "Path.hpp"
#include "ProfileFollower.hpp"
#include "RigidTransform2d.hpp"
#include "Twist2d.hpp"

namespace bns {

/**
 * A PathFollower follows a predefined path using a combination of feedforward and feedback control. It uses an
 * AdaptivePurePursuitController to choose a reference pose and generate a steering command (curvature), and then a
 * ProfileFollower to generate a profile (displacement and velocity) command.
 */
class PathFollower {
public:
	class DebugOutput {
	public:
		double t;
		double poseX;
		double poseY;
		double poseTheta;
		double linearDisplacement;
		double linearVelocity;
		double profileDisplacement;
		double profileVelocity;
		double velocityCommandDx;
		double velocityCommandDy;
		double velocityCommandDtheta;
		double steeringCommandDx;
		double steeringCommandDy;
		double steeringCommandDtheta;
		double crossTrackError;
		double alongTrackError;
		double lookaheadPointX;
		double lookaheadPointY;
		double lookaheadPointVel;
	};
	class Parameters {
	public:
		Parameters(Lookahead lookahead, double inertiaGain, double profileKp, double profileKi,
				double profileKv, double profileKffv, double profileKffa, double profileMaxVel,
				double profileMaxAcc, double goalPosTolerance, double goalVelTolerance,
				double stopSteeringDistance);
		Lookahead lookahead() const;
		double inertiaGain() const;
		double profileKp() const;
		double profileKi() const;
		double profileKv() const;
		double profileKffv() const;
		double profileKffa() const;
		double profileMaxVel() const;
		double profileMaxAcc() const;
		double goalPosTolerance() const;
		double goalVelTolerance() const;
		double stopSteeringDistance() const;
	private:
		Lookahead mLookahead;
		double mInertiaGain;
		double mProfileKp;
		double mProfileKi;
		double mProfileKv;
		double mProfileKffv;
		double mProfileKffa;
		double mProfileMaxVel;
		double mProfileMaxAcc;
		double mGoalPosTolerance;
		double mGoalVelTolerance;
		double mStopSteeringDistance;
	};
	/**
	 * Create a new PathFollower for a given path.
	 */
	PathFollower(Path path, bool reversed, Parameters parameters);
	/**
	 * Get new velocity commands to follow the path.
	 *
	 * @param t             Timestamp
	 * @param pose          Robot pose
	 * @param displacement  Robot displacement (total distance driven).
	 * @param velocity      Robot velocity.
	 * @return Velocity command to apply.
	 */
	Twist2d update(double t, RigidTransform2d pose, double displacement, double velocity);
	double crossTrackError() const;
	double alongTrackError() const;
	DebugOutput debug() const;
	bool isFinished() const;
	void forceFinish();
	bool hasPassedMarker(unsigned long marker) const;
protected:
	AdaptivePurePursuitController mSteeringController;
	Twist2d mLastSteeringDelta;
	ProfileFollower mVelocityController;
	bool mOverrideFinished = false;
	bool mDoneSteering = false;
	DebugOutput mDebugOutput;
	double mProfileMaxVel;
	double mProfileMaxAcc;
	double mCrossTrackError = 0.0;
	double mAlongTrackError = 0.0;
private:
	static constexpr double kReallyBigNumber = 1E6;
	double mGoalPosTolerance;
	double mGoalVelTolerance;
	double mInertiaGain;
	double mStopSteeringDistance;
};

}  // namespace bns

#endif  // PATHFOLLOWER_HPP_
