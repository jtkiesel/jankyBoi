#include "MotionProfileConstraints.hpp"

#include <cmath>

namespace bns {

MotionProfileConstraints::MotionProfileConstraints(double maxVel, double maxAcc) :
		mMaxVel(std::abs(maxVel)), mMaxAcc(std::abs(maxAcc)) {}

double MotionProfileConstraints::maxVel() const {
	return mMaxVel;
}

double MotionProfileConstraints::maxAcc() const {
	return mMaxAcc;
}

bool MotionProfileConstraints::equals(MotionProfileConstraints other) const {
	return (other.mMaxVel == mMaxVel) && (other.mMaxVel == mMaxVel);
}

}  // namespace bns
