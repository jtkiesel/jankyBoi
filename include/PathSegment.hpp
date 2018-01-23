#ifndef PATHSEGMENT_HPP_
#define PATHSEGMENT_HPP_

#include "MotionProfile.hpp"
#include "MotionState.hpp"
#include "Translation2d.hpp"

#include <string>

namespace bns {

/**
 * Class representing a segment of the robot's autonomous path.
 */
class PathSegment {
public:
	PathSegment();
	/**
	 * Constructor for a linear segment.
	 *
	 * @param x1        Start x.
	 * @param y1        Start y.
	 * @param x2        End x.
	 * @param y2        End y.
	 * @param maxSpeed  Maximum speed allowed on the segment.
	 */
	PathSegment(double x1, double y1, double x2, double y2, double maxSpeed, MotionState startState,
			double endSpeed);
	PathSegment(double x1, double y1, double x2, double y2, double maxSpeed, MotionState startState,
			double endSpeed, std::string marker);
	/**
	 * Constructor for an arc segment.
	 *
	 * @param x1        Start x.
	 * @param y1        Start y.
	 * @param x2        End x.
	 * @param y2        End y.
	 * @param cx        Center x.
	 * @param cy        Center y.
	 * @param maxSpeed  Maximum speed allowed on the segment.
	 */
	PathSegment(double x1, double y1, double x2, double y2, double cx, double cy, double maxSpeed,
			MotionState startState, double endSpeed);
	PathSegment(double x1, double y1, double x2, double y2, double cx, double cy, double maxSpeed,
			MotionState startState, double endSpeed, std::string marker);
	/**
	 * @return Max speed of the segment.
	 */
	double maxSpeed() const;
	void createMotionProfiler(MotionState startState, double endSpeed);
	/**
	 * @return Starting point of the segment.
	 */
	Translation2d start() const;
	/**
	 * @return End point of the segment.
	 */
	Translation2d end() const;
	/**
	 * @return  The total length of the segment.
	 */
	double length() const;
	/**
	 * Set whether or not to extrapolate the lookahead point. Should only be true for the last segment in the path.
	 *
	 * @param val
	 */
	void setExtrapolateLookahead(bool val);
	/**
	 * Gets the point on the segment closest to the robot.
	 *
	 * @param position  The current position of the robot.
	 * @return The point on the segment closest to the robot.
	 */
	Translation2d closestPoint(Translation2d position) const;
	/**
	 * Calculates the point on the segment <code>dist</code> distance from the starting point along the segment.
	 *
	 * @param dist  Distance from the starting point.
	 * @return Point on the segment <code>dist</code> distance from the starting point.
	 */
	Translation2d pointByDistance(double dist) const;
	/**
	 * Gets the remaining distance left on the segment from point <code>position</code>.
	 *
	 * @param position  Result of <code>getClosestPoint()</code>.
	 * @return Distance remaining.
	 */
	double remainingDistance(Translation2d position) const;
	double speedByDistance(double dist) const;
	double speedByClosestPoint(Translation2d robotPosition) const;
	MotionState startState() const;
	MotionState endState() const;
	std::string marker() const;
private:
	Translation2d mStart;
	Translation2d mEnd;
	Translation2d mCenter;
	Translation2d mDeltaStart;
	Translation2d mDeltaEnd;
	double mMaxSpeed;
	bool mIsLine;
	MotionProfile mSpeedController;
	bool mExtrapolateLookahead;
	std::string mMarker;
	PathSegment(double x1, double y1, double x2, double y2, double cx, double cy, double maxSpeed,
			MotionState startState, double endSpeed, bool isLine, std::string marker);
	double distanceTraveled(Translation2d robotPosition) const;
};

}  // namespace bns

#endif  // PATHSEGMENT_HPP_
