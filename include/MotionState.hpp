#ifndef MOTIONSTATE_HPP_
#define MOTIONSTATE_HPP_

namespace bns {

/**
 * A MotionState is a completely specified state of 1D motion through time.
 */
class MotionState {
public:
	static constexpr double kEpsilon = 1E-6;
	static const MotionState kInvalidState;
	MotionState(double t, double pos, double vel, double acc);
	MotionState(const MotionState &state);
	double t() const;
	double pos() const;
	double vel() const;
	double vel2() const;
	double acc() const;
	/**
	 * Extrapolates this MotionState to the specified time by applying this MotionState's acceleration.
	 *
	 * @param t  The time of the new MotionState.
	 * @return   A MotionState that is a valid predecessor (if t<=0) or successor (if t>=0) of this state.
	 */
	MotionState extrapolate(double t) const;
	/**
	 * Extrapolates this MotionState to the specified time by applying a given acceleration to the (t, pos, vel) portion
	 * of this MotionState.
	 *
	 * @param t    The time of the new MotionState.
	 * @param acc  The acceleration to apply.
	 * @return     A MotionState that is a valid predecessor (if t<=0) or successor (if t>=0) of this state (with the
	 * specified acceleration).
	 */
	MotionState extrapolate(double t, double acc) const;
	/**
	 * Find the next time (first time > MotionState.t()) that this MotionState will be at pos. This is an inverse of the
	 * extrapolate() method.
	 *
	 * @param pos  The position to query.
	 * @return     The time when we are next at pos() if we are extrapolating with a positive dt. NaN if we never reach pos.
	 */
	double nextTimeAtPos(double pos) const;
	/**
	 * Checks if two MotionStates are epsilon-equals (all fields are equal within a nominal tolerance).
	 */
	bool equals(MotionState other) const;
	/**
	 * Checks if two MotionStates are epsilon-equals (all fields are equal within a specified tolerance).
	 */
	bool equals(MotionState other, double epsilon) const;
	/**
	 * Checks if two MotionStates are coincident (t, pos, and vel are equal within a nominal tolerance, but acceleration
	 * may be different).
	 */
	bool coincident(MotionState other) const;
	/**
	 * Checks if two MotionStates are coincident (t, pos, and vel are equal within a specified tolerance, but
	 * acceleration may be different).
	 */
	bool coincident(MotionState other, double epsilon) const;
	/**
	 * Returns a MotionState that is the mirror image of this one. Pos, vel, and acc are all negated, but time is not.
	 */
	MotionState flipped() const;
protected:
	double mT;
	double mPos;
	double mVel;
	double mAcc;
};

}  // namespace bns

#endif  // MOTIONSTATE_HPP_
