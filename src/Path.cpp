#include "Path.hpp"

#include "Constants.hpp"
#include "Lookahead.hpp"
#include "MotionState.hpp"
#include "PathSegment.hpp"
#include "Translation2d.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace bns {

Path::Path() {}

void Path::extrapolateLast() {
	mSegments.back().setExtrapolateLookahead(true);
}

Translation2d Path::endPos() const {
	return mSegments.back().end();
}

void Path::addSegment(PathSegment segment) {
	mSegments.push_back(segment);
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
