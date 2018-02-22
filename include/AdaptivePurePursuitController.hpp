#ifndef ADAPTIVEPUREPURSUITCONTROLLER_HPP_
#define ADAPTIVEPUREPURSUITCONTROLLER_HPP_

#include "Lookahead.hpp"
#include "Path.hpp"
#include "RigidTransform2d.hpp"
#include "Translation2d.hpp"
#include "Twist2d.hpp"

namespace bns {

/**
 * Implements an adaptive pure pursuit controller. See:
 * https://www.ri.cmu.edu/pub_files/pub1/kelly_alonzo_1994_4/kelly_alonzo_1994_4.pdf
 *
 * Basically, we find a spot on the path we'd like to follow and calculate the
 * arc necessary to make us land on that spot. The target spot is a specified
 * distance ahead of us, and we look further ahead the greater our tracking
 * error. We also return the maximum speed we'd like to be going when we reach
 * the target spot.
 */
class AdaptivePurePursuitController {
public:
	class Command {
	public:
		Twist2d delta;
		double crossTrackError;
		double maxVel;
		double endVel;
		Translation2d lookaheadPoint;
		double remainingPathLength;
		Command();
		Command(Twist2d delta, double crossTrackError, double maxVel, double endVel,
				Translation2d lookaheadPoint, double remainingPathLength);
	};
	AdaptivePurePursuitController(Path path, bool reversed, Lookahead lookahead);
	/**
	 * Gives the Command that the robot should take to follow the path.
	 *
	 * @param pose  Robot pose.
	 * @return Movement command for the robot to follow.
	 */
	Command update(RigidTransform2d pose);
	bool hasPassedMarker(unsigned long marker) const;
	class Arc {
	public:
		Translation2d center;
		double radius;
		double length;
		Arc(RigidTransform2d pose, Translation2d point);
	};
	/**
	 * Gives the center of the circle joining the lookahead point and robot
	 * pose.
	 *
	 * @param pose   Robot pose.
	 * @param point  Lookahead point.
	 * @return Center of the circle joining the lookahead point and robot pose.
	 */
	static Translation2d center(RigidTransform2d pose, Translation2d point);
	/**
	 * Gives the radius of the circle joining the lookahead point and robot
	 * pose.
	 *
	 * @param pose   Robot pose.
	 * @param point  Lookahead point.
	 * @return Radius of the circle joining the lookahead point and robot pose.
	 */
	static double radius(RigidTransform2d pose, Translation2d point);
	/**
	 * Gives the length of the arc joining the lookahead point and robot pose (assuming forward motion).
	 *
	 * @param pose   Robot pose.
	 * @param point  Lookahead point.
	 * @return Length of the arc joining the lookahead point and robot pose.
	 */
	static double length(RigidTransform2d pose, Translation2d point);
	static double length(RigidTransform2d pose, Translation2d point, Translation2d center, double radius);
	/**
	 * Gives the direction the robot should turn to stay on the path.
	 *
	 * @param pose   Robot pose.
	 * @param point  Lookahead point.
	 * @return Direction the robot should turn: -1 is left, +1 is right.
	 */
	static int direction(RigidTransform2d pose, Translation2d point);
	/**
	 * @return Has the robot reached the end of the path?
	 */
	bool isFinished() const;
protected:
	Path mPath;
	bool mAtEndOfPath;
private:
	static constexpr double kReallyBigNumber = 1E6;
	static constexpr double kEpsilon = 1E-6;
	bool mReversed;
	Lookahead mLookahead;
};

}  // namespace bns

#endif  // ADAPTIVEPUREPURSUITCONTROLLER_HPP_
