#ifndef MOTIONPROFILE_HPP_
#define MOTIONPROFILE_HPP_

#include "MotionSegment.hpp"
#include "MotionState.hpp"
#include "Optional.hpp"

#include <vector>

namespace bns {

/**
 * A motion profile specifies a 1D time-parameterized trajectory. The trajectory is composed of successively coincident
 * MotionSegments from which the desired state of motion at any given distance or time can be calculated.
 */
class MotionProfile {
public:
	/**
	 * Create an empty MotionProfile.
	 */
	MotionProfile();
	/**
	 * Create a MotionProfile from an existing list of segments (note that validity is not checked).
	 *
	 * @param segments  The new segments of the profile.
	 */
	MotionProfile(std::vector<MotionSegment> segments);
	/**
	 * Checks if the given MotionProfile is valid. This checks that:
	 * 1. All segments are valid.
	 * 2. Successive segments are C1 continuous in position and C0 continuous in velocity.
	 *
	 * @return <code>true</code> if the MotionProfile is valid.
	 */
	bool isValid() const;
	/**
	 * Check if the profile is empty.
	 *
	 * @return  <code>true</code> if there are no segments.
	 */
	bool isEmpty() const;
	/**
	 * Get the interpolated MotionState at any given time.
	 *
	 * @param t  The time to query.
	 * @return Empty if the time is outside the time bounds of the profile, or the resulting MotionState otherwise.
	 */
	Optional<MotionState> stateByTime(double t) const;
	/**
	 * Get the interpolated MotionState at any given time, clamping to the endpoints if time is out of bounds.
	 *
	 * @param t  The time to query.
	 * @return The MotionState at time t, or closest to it if t is outside the profile.
	 */
	MotionState stateByTimeClamped(double t) const;
	/**
	 * Get the interpolated MotionState by distance (the "pos()" field of MotionState). Note that since a profile may
	 * reverse, this method only returns the *first* instance of this position.
	 *
	 * @param pos  The position to query.
	 * @return Empty if the profile never crosses pos or if the profile is invalid, or the resulting MotionState
	 * otherwise.
	 */
	Optional<MotionState> firstStateByPos(double pos) const;
	/**
	 * Remove all parts of the profile prior to the query time. This eliminates whole segments and also shortens any
	 * segments containing t.
	 *
	 * @param t  The query time.
	 */
	void trimBeforeTime(double t);
	/**
	 * Remove all segments.
	 */
	void clear();
	/**
	 * Remove all segments and initialize to the desired state (actually a segment of length 0 that starts and ends at
	 * initialState).
	 *
	 * @param initialState  The MotionState to initialize to.
	 */
	void reset(MotionState initialState);
	/**
	 * Remove redundant segments (segments whose start and end states are coincident).
	 */
	void consolidate();
	/**
	 * Add to the profile by applying an acceleration control for a given time. This is appended to the previous last
	 * state.
	 *
	 * @param acc  The acceleration to apply.
	 * @param dt   The period of time to apply the given acceleration.
	 */
	void appendControl(double acc, double dt);
	/**
	 * Add to the profile by inserting a new segment. No validity checking is done.
	 *
	 * @param segment  The segment to add.
	 */
	void appendSegment(MotionSegment segment);
	/**
	 * Add to the profile by inserting a new profile after the final state. No validity checking is done.
	 *
	 * @param profile  The profile to add.
	 */
	void appendProfile(MotionProfile profile);
	/**
	 * @return The number of segments.
	 */
	int size() const;
	/**
	 * @return The list of segments.
	 */
	std::vector<MotionSegment> segments() const;
	/**
	 * @return The first state in the profile (or kInvalidState if empty).
	 */
	MotionState startState() const;
	/**
	 * @return The time of the first state in the profile (or NaN if empty).
	 */
	double startTime() const;
	/**
	 * @return The pos of the first state in the profile (or NaN if empty).
	 */
	double startPos() const;
	/**
	 * @return The last state in the profile (or kInvalidState if empty).
	 */
	MotionState endState() const;
	/**
	 * @return The time of the last state in the profile (or NaN if empty).
	 */
	double endTime() const;
	/**
	 * @return The pos of the last state in the profile (or NaN if empty).
	 */
	double endPos() const;
	/**
	 * @return The duration of the entire profile.
	 */
	double duration() const;
	/**
	 * @return The total distance covered by the profile. Note that distance is the sum of absolute distances of all
	 * segments, so a reversing profile will count the distance covered in each direction.
	 */
	double length() const;
protected:
	std::vector<MotionSegment> mSegments;
private:
	static constexpr double kEpsilon = 1E-6;
};

}  // namespace bns

#endif  // MOTIONPROFILE_HPP_
