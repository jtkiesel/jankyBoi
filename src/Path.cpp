#include "Path.hpp"

#include "api.hpp"
#include "Constants.hpp"
#include "Lookahead.hpp"
#include "MotionState.hpp"
#include "PathSegment.hpp"
#include "RigidTransform2d.hpp"
#include "Translation2d.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace bns {

Path::Waypoint::Waypoint(double x, double y, double radius, double vel) :
		Waypoint(Translation2d(x, y), radius, vel, "") {}

Path::Waypoint::Waypoint(double x, double y, double radius, double vel, std::string marker) :
		Waypoint(Translation2d(x, y), radius, vel, marker) {}

Path::Waypoint::Waypoint(Translation2d pos, double radius, double vel) :
		Waypoint(pos, radius, vel, "") {}

Path::Waypoint::Waypoint(Translation2d pos, double radius, double vel, std::string marker) :
		mPos(pos), mRadius(radius), mVel(vel), mMarker(marker) {}

Path::Waypoint::Waypoint(const Waypoint &other) :
		Waypoint(other.mPos, other.mRadius, other.mVel, other.mMarker) {}

Translation2d Path::Waypoint::pos() const {
	return mPos;
}

double Path::Waypoint::radius() const {
	return mRadius;
}

double Path::Waypoint::vel() const {
	return mRadius;
}

std::string Path::Waypoint::marker() const {
	return mMarker;
}

Path::Line::Line(Waypoint pointA, Waypoint pointB) : mPointA(pointA), mPointB(pointB),
		mSlope(Translation2d(pointA.pos(), pointB.pos())),
		mStart(pointA.pos().translateBy(mSlope.scale(pointA.radius() / mSlope.norm()))),
		mEnd(pointB.pos().translateBy(mSlope.scale(-pointB.radius() / mSlope.norm()))), mVel(pointB.vel()) {}

Path::Waypoint Path::Line::pointA() const {
	return mPointA;
}

Path::Waypoint Path::Line::pointB() const {
	return mPointB;
}

Translation2d Path::Line::start() const {
	return mStart;
}

Translation2d Path::Line::end() const {
	return mEnd;
}

Translation2d Path::Line::slope() const {
	return mSlope;
}

Path::Arc::Arc(Waypoint pointA, Waypoint pointB, Waypoint pointC) :
		Arc(Line(pointA, pointB), Line(pointB, pointC)) {}

Path::Arc::Arc(Line lineA, Line lineB) : mLineA(lineA), mLineB(lineB),
		mCenter(intersect(lineA, lineB)), mRadius(Translation2d(mCenter, lineA.end()).norm()),
		mVel((lineA.vel() + lineB.vel()) / 2.0) {}

Path::Line Path::Arc::lineA() const {
	return mLineA;
}

Path::Line Path::Arc::lineB() const {
	return mLineB;
}

Translation2d Path::Arc::center() const {
	return mCenter;
}

double Path::Arc::radius() const {
	return mRadius;
}

double Path::Arc::vel() const {
	return mVel;
}

Translation2d Path::Arc::intersect(Line lineA, Line lineB) {
	const RigidTransform2d line1 = RigidTransform2d(lineA.end(), Rotation2d(lineA.slope(), true).normal());
	const RigidTransform2d line2 = RigidTransform2d(lineB.start(), Rotation2d(lineB.slope(), true).normal());
	return line1.intersection(line2);
}

Path::Path() {}

Path::Path(std::vector<Path::Waypoint> waypoints) {
	int size = waypoints.size();
	if (size) {
		pros::printf("Error: Path must contain at lease 2 waypoints.\n");
		return;
	}
	for (int i = 0; i < size - 2; i++) {
		addArc(Arc(waypoints.at(i), waypoints.at(i + 1), waypoints.at(i + 2)));
	}
	addLine(Line(waypoints.at(size - 2), waypoints.at(size - 1)), 0.0);
	extrapolateLast();
	verifyVels();
}

void Path::extrapolateLast() {
	mSegments.back().setExtrapolateLookahead(true);
}

Translation2d Path::endPos() const {
	return mSegments.back().end();
}

void Path::addSegment(PathSegment segment) {
	mSegments.push_back(segment);
}

void Path::addLine(Line line, double endVel) {
	double pathLength = Translation2d(line.end(), line.start()).norm();
	if (pathLength > kEpsilon) {
		if (!line.pointB().marker().empty()) {
			addSegment(PathSegment(line.start().x(), line.start().y(), line.end().x(), line.end().y(),
					line.pointB().vel(), lastMotionState(), endVel, line.pointB().marker()));
		} else {
			addSegment(PathSegment(line.start().x(), line.start().y(), line.end().x(), line.end().y(),
					line.pointB().vel(), lastMotionState(), endVel));
		}
	}
}

void Path::addArc(Arc arc) {
	addLine(arc.lineA(), arc.vel());
	if (arc.radius() > kEpsilon && arc.radius() < kReallyBigNumber) {
		addSegment(PathSegment(arc.lineA().end().x(), arc.lineA().end().y(), arc.lineB().start().x(),
				arc.lineB().start().y(), arc.center().x(), arc.center().y(), arc.vel(), lastMotionState(),
				arc.lineB().vel()));
	}
}

MotionState Path::lastMotionState() const {
	if (mSegments.empty()) {
		return MotionState(0.0, 0.0, 0.0, 0.0);
	}
	return MotionState(0.0, 0.0, mSegments.back().endState().vel(), mSegments.back().endState().acc());
}

double Path::segmentRemainingDist(Translation2d robotPos) const {
	return mSegments.front().remainingDistance(mSegments.front().closestPoint(robotPos));
}

double Path::segmentLength() const {
	return mSegments.front().length();
}

Path::TargetPointReport Path::targetPoint(Translation2d robotPos, Lookahead lookahead) {
	TargetPointReport targetPoint = TargetPointReport();
	PathSegment currentSegment = mSegments.front();
	targetPoint.closestPoint = currentSegment.closestPoint(robotPos);
	targetPoint.closestPointDist = Translation2d(robotPos, targetPoint.closestPoint).norm();
	targetPoint.remainingSegmentDist = currentSegment.remainingDistance(targetPoint.closestPoint);
	targetPoint.remainingPathDist = targetPoint.remainingSegmentDist;
	for (auto it = mSegments.begin() + 1; it < mSegments.end(); it++) {
		targetPoint.remainingPathDist += it->length();
	}
	targetPoint.closestPointVel = currentSegment.speedByDistance(currentSegment.length() - targetPoint.remainingSegmentDist);
	double lookaheadDist = lookahead.lookaheadByVel(targetPoint.closestPointVel) + targetPoint.closestPointDist;
	if (targetPoint.remainingSegmentDist < lookaheadDist && mSegments.size() > 1) {
		lookaheadDist -= targetPoint.remainingSegmentDist;
		for (auto it = mSegments.begin() + 1; it < mSegments.end(); it++) {
			const double length = it->length();
			if (length >= lookaheadDist || it == mSegments.end() - 1) {
				break;
			}
			lookaheadDist -= length;
		}
	} else {
		lookaheadDist += (currentSegment.length() - targetPoint.remainingSegmentDist);
	}
	targetPoint.maxVel = currentSegment.maxSpeed();
	targetPoint.lookaheadPoint = currentSegment.pointByDistance(lookaheadDist);
	targetPoint.lookaheadPointVel = currentSegment.speedByDistance(lookaheadDist);
	checkSegmentDone(targetPoint.closestPoint);
	return targetPoint;
}

double Path::velByPos(Translation2d robotPos) const {
	return mSegments.front().speedByClosestPoint(robotPos);
}

void Path::checkSegmentDone(Translation2d robotPos) {
	double remainingDist = mSegments.front().remainingDistance(mSegments.front().closestPoint(robotPos));
	if (remainingDist < Constants::kSegmentCompletionTolerance) {
		removeCurrentSegment();
	}
}

void Path::removeCurrentSegment() {
	std::string marker = mSegments.front().marker();
	mSegments.pop_front();
	if (!marker.empty()) {
		mCrossedMarkers.insert(marker);
	}
}

void Path::verifyVels() {
	const int size = mSegments.size();
	double maxStartVel = 0.0;
	std::vector<double> startVels(size + 1);
	startVels.at(size) = 0.0;
	for (int i = size - 1; i >= 0; i--) {
		maxStartVel += std::sqrt(maxStartVel * maxStartVel
				+ 2.0 * Constants::kPathFollowingMaxAccel * mSegments.at(i).length());
		startVels.at(i) = std::min(mSegments.at(i).startState().vel(), maxStartVel);
		maxStartVel = startVels.at(i);
	}
	for (int i = 0; i < size; i++) {
		MotionState startState = (i > 0) ? mSegments.at(i - 1).endState() : MotionState(0.0, 0.0, 0.0, 0.0);
		startState = MotionState(0.0, 0.0, startState.vel(), startState.acc());
		mSegments.at(i).createMotionProfiler(startState, startVels.at(i + 1));
	}
}

bool Path::hasPassedMarker(std::string marker) const {
	return mCrossedMarkers.find(marker) != mCrossedMarkers.end();
}

}  // namespace bns
