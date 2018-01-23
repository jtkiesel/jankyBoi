#ifndef PATH_HPP_
#define PATH_HPP_

#include "Lookahead.hpp"
#include "MotionState.hpp"
#include "PathSegment.hpp"
#include "Translation2d.hpp"

#include <deque>
#include <string>
#include <unordered_set>

namespace bns {

/**
 * Class representing the robot's autonomous path.
 *
 * Field Coordinate System: Uses a right hand coordinate system. Positive x is
 * right, positive y is up, and the origin is at the bottom left corner of the
 * field. For angles, 0 degrees is facing right (1, 0) and angles increase as
 * you turn counter clockwise.
 */
class Path {
public:
	Path();
	void extrapolateLast();
	Translation2d endPos() const;
	/**
	 * Add a segment to the Path.
	 *
	 * @param segment  The segment to add.
	 */
	void addSegment(PathSegment segment);
	/**
	 * @return The last MotionState in the Path.
	 */
	MotionState lastMotionState() const;
	/**
	 * Get the remaining distance left for the robot to travel on the current
	 * segment.
	 *
	 * @param robotPos  The robot position.
	 * @return Remaining distance on current segment.
	 */
	double segmentRemainingDist(Translation2d robotPos) const;
	/**
	 * @return The length of the current segment.
	 */
	double segmentLength() const;
	class TargetPointReport {
	public:
		Translation2d closestPoint;
		double closestPointDist;
		double closestPointVel;
		Translation2d lookaheadPoint;
		double maxVel;
		double lookaheadPointVel;
		double remainingSegmentDist;
		double remainingPathDist;
		TargetPointReport() {};
	};
	/**
	 * Gives the position of the lookahead point (and removes any segments prior
	 * to this point).
	 *
	 * @param robot  Translation of the current robot pose.
	 * @return Report containing everything we might want to know about the
	 * target point.
	 */
	TargetPointReport targetPoint(Translation2d robotPos, Lookahead lookahead);
	/**
	 * Gives the velocity the robot should be traveling at the given position.
	 *
	 * @param robotPos  Position of the robot.
	 * @return Velocity robot should be traveling.
	 */
	double velByPos(Translation2d robotPos) const;
	/**
	 * Checks if the robot has finished traveling along the current segment then
	 * removes it from the path if it has.
	 *
	 * @param robotPos  Robot position.
	 */
	void checkSegmentDone(Translation2d robotPos);
	void removeCurrentSegment();
	/**
	 * Ensures that all velocities in the path are attainable and robot can slow
	 * down in time.
	 */
	void verifyVels();
	bool hasPassedMarker(std::string marker) const;
protected:
	std::deque<PathSegment> mSegments;
	PathSegment mPrevSegment;
	std::unordered_set<std::string> mCrossedMarkers;
};

}  // namespace bns

#endif  // PATH_HPP_
