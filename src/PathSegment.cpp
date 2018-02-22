#include "PathSegment.hpp"

#include "api.hpp"
#include "Constants.hpp"
#include "MotionProfileConstraints.hpp"
#include "MotionProfileGenerator.hpp"
#include "MotionProfileGoal.hpp"
#include "MotionState.hpp"

namespace bns {

PathSegment::PathSegment() {}

PathSegment::PathSegment(double x1, double y1, double x2, double y2, double maxSpeed,
		MotionState startState, double endSpeed) :
		PathSegment(x1, y1, x2, y2, 0.0, 0.0, maxSpeed, startState, endSpeed, false, 0) {}

PathSegment::PathSegment(double x1, double y1, double x2, double y2, double maxSpeed,
		MotionState startState, double endSpeed, unsigned long marker) :
		PathSegment(x1, y1, x2, y2, 0.0, 0.0, maxSpeed, startState, endSpeed, false, marker) {}

PathSegment::PathSegment(double x1, double y1, double x2, double y2, double cx, double cy,
		double maxSpeed, MotionState startState, double endSpeed) :
		PathSegment(x1, y1, x2, y2, cx, cy, maxSpeed, startState, endSpeed, false, 0) {}

PathSegment::PathSegment(double x1, double y1, double x2, double y2, double cx, double cy,
		double maxSpeed, MotionState startState, double endSpeed, unsigned long marker) :
		PathSegment(x1, y1, x2, y2, cx, cy, maxSpeed, startState, endSpeed, false, marker) {}

PathSegment::PathSegment(double x1, double y1, double x2, double y2, double cx, double cy,
		double maxSpeed, MotionState startState, double endSpeed, bool isLine, unsigned long marker) :
		mStart(Translation2d(x1, y1)), mEnd(Translation2d(x2, y2)), mCenter(Translation2d(cx, cy)),
		mDeltaStart(Translation2d(mCenter, mStart)), mDeltaEnd(Translation2d(mCenter, mEnd)),
		mMaxSpeed(maxSpeed), mIsLine(isLine), mExtrapolateLookahead(false), mMarker(marker) {
	createMotionProfiler(startState, endSpeed);
}

double PathSegment::maxSpeed() const {
	return mMaxSpeed;
}

void PathSegment::createMotionProfiler(MotionState startState, double endSpeed) {
	MotionProfileConstraints motionConstraints = MotionProfileConstraints(mMaxSpeed, Constants::kPathFollowingMaxAccel);
	MotionProfileGoal goalState = MotionProfileGoal(length(), endSpeed);
	mSpeedController = MotionProfileGenerator::generateProfile(motionConstraints, goalState, startState);
}

Translation2d PathSegment::start() const {
	return mStart;
}

Translation2d PathSegment::end() const {
	return mEnd;
}

double PathSegment::length() const {
	if (mIsLine) {
		return mDeltaStart.norm();
	}
	return mDeltaStart.norm() * Translation2d::rotation(mDeltaStart, mDeltaEnd).radians();
}

void PathSegment::setExtrapolateLookahead(bool val) {
	mExtrapolateLookahead = val;
}

Translation2d PathSegment::closestPoint(Translation2d position) const {
	if (mIsLine) {
		Translation2d delta = Translation2d(mStart, mEnd);
		double u = ((position.x() - mStart.x()) * delta.x() + (position.y() - mStart.y()) * delta.y())
				/ (delta.x() * delta.x() + delta.y() * delta.y());
		if (u >= 0.0 && u <= 1.0) {
			return Translation2d(mStart.x() + u * delta.x(), mStart.y() + u * delta.y());
		}
		return (u < 0.0) ? mStart : mEnd;
	}
	Translation2d deltaPosition = Translation2d(mCenter, position);
	deltaPosition = deltaPosition.scale(mDeltaStart.norm() / deltaPosition.norm());
	if (Translation2d::cross(deltaPosition, mDeltaStart) * Translation2d::cross(deltaPosition, mDeltaEnd) < 0.0) {
		return mCenter.translateBy(deltaPosition);
	}
	Translation2d startDist = Translation2d(position, mStart);
	Translation2d endDist = Translation2d(position, mEnd);
	return (endDist.norm() < startDist.norm()) ? mEnd : mStart;
}

Translation2d PathSegment::pointByDistance(double dist) const {
	double length = this->length();
	if (!mExtrapolateLookahead && dist > length) {
		dist = length;
	}
	if (mIsLine) {
		return mStart.translateBy(mDeltaStart.scale(dist / length));
	}
	double deltaAngle = Translation2d::rotation(mDeltaStart, mDeltaEnd).radians()
			* ((Translation2d::cross(mDeltaStart, mDeltaEnd) >= 0.0) ? 1 : -1) * (dist / length);
	Translation2d t = mDeltaStart.rotateBy(Rotation2d::fromRadians(deltaAngle));
	return mCenter.translateBy(t);
}

double PathSegment::remainingDistance(Translation2d position) const {
	if (mIsLine) {
		return Translation2d(mEnd, position).norm();
	}
	Translation2d deltaPosition = Translation2d(mCenter, position);
	double angle = Translation2d::rotation(mDeltaEnd, deltaPosition).radians();
	double totalAngle = Translation2d::rotation(mDeltaStart, mDeltaEnd).radians();
	return angle / totalAngle * length();
}

double PathSegment::distanceTraveled(Translation2d robotPosition) const {
	Translation2d pathPosition = closestPoint(robotPosition);
	double remainingDist = remainingDistance(pathPosition);
	return length() - remainingDist;
}

double PathSegment::speedByDistance(double dist) const {
	if (dist < mSpeedController.startPos()) {
		dist = mSpeedController.startPos();
	} else if (dist > mSpeedController.endPos()) {
		dist = mSpeedController.endPos();
	}
	Optional<MotionState> state = mSpeedController.firstStateByPos(dist);
	if (state.isPresent()) {
		return state.get().vel();
	}
	pros::printf("Velocity does not exist at that position!");
	return 0.0;
}

double PathSegment::speedByClosestPoint(Translation2d robotPosition) const {
	return speedByDistance(distanceTraveled(robotPosition));
}

MotionState PathSegment::startState() const {
	return mSpeedController.startState();
}

MotionState PathSegment::endState() const {
	return mSpeedController.endState();
}

unsigned long PathSegment::marker() const {
	return mMarker;
}

}  // namespace bns
