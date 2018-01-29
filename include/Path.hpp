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
	/**
	 * A waypoint along a path. Contains a position, radius (for creating curved
	 * paths), and velocity. The information from these waypoints is used by the
	 * PathBuilder class to generate Paths. Waypoints also contain an optional
	 * marker that is used by the WaitForPathMarkerAction.
	 *
	 * @see PathBuilder
	 * @see WaitForPathMarkerAction
	 */
	class Waypoint {
	public:
		Waypoint(double x, double y, double radius, double vel);
		Waypoint(double x, double y, double radius, double vel, std::string marker);
		Waypoint(Translation2d pos, double radius, double vel);
		Waypoint(Translation2d pos, double radius, double vel, std::string marker);
		Waypoint(const Waypoint &other);
		Translation2d pos() const;
		double radius() const;
		double vel() const;
		std::string marker() const;
	protected:
		Translation2d mPos;
		double mRadius;
		double mVel;
		std::string mMarker;
	};
	/**
	 * A Line is formed by two Waypoints. Contains a start and end position,
	 * slope, and velocity.
	 */
	class Line {
	public:
		Line(Waypoint a, Waypoint b);
		Waypoint pointA() const;
		Waypoint pointB() const;
		Translation2d start() const;
		Translation2d end() const;
		Translation2d slope() const;
		double vel() const;
	protected:
		Waypoint mPointA;
		Waypoint mPointB;
		Translation2d mSlope;
		Translation2d mStart;
		Translation2d mEnd;
		double mVel;
	private:
		void addToPath(Path path, double endVel);
	};
	/**
	 * An Arc is formed by two Lines that share a common Waypoint. Contains a
	 * center position, radius, and velocity.
	 */
	class Arc {
	public:
		Arc(Waypoint pointA, Waypoint pointB, Waypoint pointC);
		Arc(Line lineA, Line lineB);
		Line lineA() const;
		Line lineB() const;
		Translation2d center() const;
		double radius() const;
		double vel() const;
	protected:
		Line mLineA;
		Line mLineB;
		Translation2d mCenter;
		double mRadius;
		double mVel;
	private:
		static Translation2d intersect(Line lineA, Line lineB);
	};
	Path();
	Path(std::vector<Waypoint> waypoints);
	void extrapolateLast();
	Translation2d endPos() const;
	/**
	 * Add a segment to the Path.
	 *
	 * @param segment  The segment to add.
	 */
	void addSegment(PathSegment segment);
	void addLine(Line line, double endVel);
	void addArc(Arc arc);
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
private:
	static constexpr double kEpsilon = 1E-9;
	static constexpr double kReallyBigNumber = 1E9;
};

}  // namespace bns

#endif  // PATH_HPP_
