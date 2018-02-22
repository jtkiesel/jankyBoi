#include "PathFollowerTest.hpp"

#include "api.hpp"
#include "MotionProfileConstraints.hpp"
#include "PathFollower.hpp"
#include "ProfileFollower.hpp"
#include "Twist2d.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace bns {

void testPathFollower() {
	const PathFollower::Parameters kParameters = PathFollower::Parameters(
			Lookahead(16.0, 16.0, 0.0, 120.0),
			0.0,  // Inertia gain.
			0.75,  // Profile Kp.
			0.03,  // Profile Ki.
			0.02,  // Profile Kv.
			1.0,  // Profile Kffv.
			0.0,  // Profile Kffa.
			Constants::kPathFollowingMaxVel, Constants::kPathFollowingMaxAccel,
			Constants::kPathFollowingGoalPosTolerance, Constants::kPathFollowingGoalVelTolerance,
			Constants::kPathStopSteeringDistance);
	const double kVel = 3.0;
	const RigidTransform2d kStartPose = RigidTransform2d(Translation2d(30.0, 20.0), Rotation2d());
	std::vector<Path::Waypoint> waypoints;
	waypoints.push_back(Path::Waypoint(kStartPose.translation(), 0.0, kVel));
	waypoints.push_back(Path::Waypoint(60.0, 0.0, 10.0, kVel));
	waypoints.push_back(Path::Waypoint(90.0, 20.0, 0.0, kVel));
	PathFollower controller = PathFollower(Path(waypoints), false, kParameters);

	const double dt = 0.01;
	RigidTransform2d robotPose = kStartPose;
	double t = 0.0;
	double displacement = 0.0;
	double velocity = 0.0;
	while (!controller.isFinished() && t < 10.0) {
		// Follow the path.
		Twist2d command = controller.update(t, robotPose, displacement, velocity);
		robotPose = robotPose.transformBy(RigidTransform2d::exp(command.scaled(dt)));

		t += dt;
		const double prevVel = velocity;
		velocity = command.dx();
		displacement += velocity * dt;

		pros::printf("t %.2f, displacement %.2f, lin vel %.2f, lin acc %.2f, ang vel %.2f, pose T: (%.2f, %.2f) R: (%.2f), CTE %.2f, ATE %.2f\n",
				t, displacement, (velocity - prevVel) / dt, command.dtheta(), robotPose.translation().x(), robotPose.translation().y(), robotPose.rotation().degrees(), controller.crossTrackError(), controller.alongTrackError());
	}
	pros::printf("T: (%.2f, %.2f) R: (%.2f)\n", robotPose.translation().x(), robotPose.translation().y(), robotPose.rotation().degrees());
	if (!controller.isFinished()) {
		pros::printf("ERROR: Not finished.\n");
	}
	if (controller.alongTrackError() >= 1.0) {
		pros::printf("ERROR: Along track error too great.\n");
	}
	if (controller.crossTrackError() >= 1.0) {
		pros::printf("ERROR: Cross track error too great.\n");
	}
}

}  // namespace bns
