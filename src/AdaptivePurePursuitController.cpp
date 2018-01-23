#include "AdaptivePurePursuitController.hpp"

#include "Lookahead.hpp"
#include "Path.hpp"
#include "RigidTransform2d.hpp"
#include "Rotation2d.hpp"
#include "Translation2d.hpp"
#include "Twist2d.hpp"
#include "util.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace bns {

AdaptivePurePursuitController::Command::Command() {}

AdaptivePurePursuitController::Command::Command(Twist2d delta, double crossTrackError,
		double maxVel, double endVel, Translation2d lookaheadPoint, double remainingPathLength) :
		delta(delta), crossTrackError(crossTrackError), maxVel(maxVel), endVel(endVel),
		lookaheadPoint(lookaheadPoint), remainingPathLength(remainingPathLength) {}

AdaptivePurePursuitController::AdaptivePurePursuitController(Path path, bool reversed,
		Lookahead lookahead) : mPath(path), mAtEndOfPath(false), mReversed(reversed),
		mLookahead(lookahead) {}

AdaptivePurePursuitController::Command AdaptivePurePursuitController::update(RigidTransform2d pose) {
	if (mReversed) {
		pose = RigidTransform2d(pose.translation(), pose.rotation().rotateBy(Rotation2d::fromRadians(kPi)));
	}
	const Path::TargetPointReport report = mPath.targetPoint(pose.translation(), mLookahead);
	if (mAtEndOfPath) {
		// Stop.
		return Command(Twist2d::identity(), report.closestPointDist, report.maxVel, 0.0,
				report.lookaheadPoint, report.remainingPathDist);
	}
	const Arc arc = Arc(pose, report.lookaheadPoint);
	double scaleFactor = 1.0;
	// Ensure we don't overshoot the end of the path (once the lookahead
	// velocity drops to zero).
	if (report.lookaheadPointVel < kEpsilon && report.remainingPathDist < arc.length) {
		scaleFactor = std::max(0.0, report.remainingPathDist / arc.length);
		mAtEndOfPath = true;
	} else {
		mAtEndOfPath = false;
	}
	if (mReversed) {
		scaleFactor *= -1;
	}
	return Command(Twist2d(scaleFactor * arc.length, 0.0,
			arc.length * direction(pose, report.lookaheadPoint) * std::abs(scaleFactor) / arc.radius),
			report.closestPointDist, report.maxVel, report.lookaheadPointVel * signum(scaleFactor),
			report.lookaheadPoint, report.remainingPathDist);
}

bool AdaptivePurePursuitController::hasPassedMarker(std::string marker) const {
	return mPath.hasPassedMarker(marker);
}

AdaptivePurePursuitController::Arc::Arc(RigidTransform2d pose, Translation2d point) :
		center(AdaptivePurePursuitController::center(pose, point)),
		radius(Translation2d(center, point).norm()),
		length(AdaptivePurePursuitController::length(pose, point, center, radius)) {}

Translation2d AdaptivePurePursuitController::center(RigidTransform2d pose, Translation2d point) {
	const Translation2d poseToPointHalfway = pose.translation().interpolate(point, 0.5);
	const Rotation2d normal = pose.translation().inverse().translateBy(poseToPointHalfway).direction().normal();
	const RigidTransform2d perpendicularBisector = RigidTransform2d(poseToPointHalfway, normal);
	const RigidTransform2d normalFromPose = RigidTransform2d(pose.translation(), pose.rotation().normal());
	if (normalFromPose.isColinear(perpendicularBisector.normal())) {
		// Special case: center is poseToPointHalfway.
		return poseToPointHalfway;
	}
	return normalFromPose.intersection(perpendicularBisector);
}

double AdaptivePurePursuitController::radius(RigidTransform2d pose, Translation2d point) {
	return Translation2d(center(pose, point), point).norm();
}

double AdaptivePurePursuitController::length(RigidTransform2d pose, Translation2d point) {
	return length(pose, point, center(pose, point), radius(pose, point));
}

double AdaptivePurePursuitController::length(RigidTransform2d pose, Translation2d point,
		Translation2d center, double radius) {
	if (radius < kReallyBigNumber) {
		// If the point is behind the pose, we want the opposite of this angle.
		// To determine if the point is behind, check the sign of the cross-
		// product between the normal vector and the vector from pose to point.
		const bool isBehind = signum(Translation2d::cross(pose.rotation().normal().toTranslation(),
				Translation2d(pose.translation(), point))) > 0.0;
		const Rotation2d angle = Translation2d::rotation(Translation2d(center, point),
				Translation2d(center, pose.translation()));
		return radius * (isBehind ? 2.0 * kPi - std::abs(angle.radians()) : std::abs(angle.radians()));
	}
	return Translation2d(pose.translation(), point).norm();
}

int AdaptivePurePursuitController::direction(RigidTransform2d pose, Translation2d point) {
	Translation2d poseToPoint = Translation2d(pose.translation(), point);
	Translation2d robot = pose.rotation().toTranslation();
	// If robot < pose, turn left.
	return ((robot.x() * poseToPoint.y() - robot.y() * poseToPoint.x()) < 0) ? -1 : 1;
}

bool AdaptivePurePursuitController::isFinished() const {
	return mAtEndOfPath;
}

}  // namespace bns
