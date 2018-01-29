#ifndef MOTIONPROFILECONSTRAINTS_HPP_
#define MOTIONPROFILECONSTRAINTS_HPP_

namespace bns {

/**
 * Constraints for constructing a MotionProfile.
 */
class MotionProfileConstraints {
public:
	MotionProfileConstraints();
	MotionProfileConstraints(double maxVel, double maxAcc);
	/**
	 * @return The (positive) maximum allowed velocity.
	 */
	double maxVel() const;
	/**
	 * @return The (positive) maximum allowed acceleration.
	 */
	double maxAcc() const;
	bool equals(MotionProfileConstraints other) const;
protected:
	double mMaxVel;
	double mMaxAcc;
};

}  // namespace bns

#endif  // MOTIONPROFILECONSTRAINTS_HPP_
