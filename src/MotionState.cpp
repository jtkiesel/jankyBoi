#include "MotionState.hpp"

#include "util.hpp"

#include <cmath>

namespace bns {

const MotionState MotionState::kInvalidState = MotionState(kNaN, kNaN, kNaN, kNaN);

MotionState::MotionState() : MotionState(kInvalidState) {};

MotionState::MotionState(double t, double pos, double vel, double acc) : mT(t), mPos(pos), mVel(vel), mAcc(acc) {}

MotionState::MotionState(const MotionState &other) : MotionState(other.mT, other.mPos, other.mVel, other.mAcc) {}

double MotionState::t() const {
	return mT;
}

double MotionState::pos() const {
	return mPos;
}

double MotionState::vel() const {
	return mVel;
}

double MotionState::vel2() const {
	return mVel * mVel;
}

double MotionState::acc() const {
	return mAcc;
}

MotionState MotionState::extrapolate(double t) const {
	return extrapolate(t, mAcc);
}

MotionState MotionState::extrapolate(double t, double acc) const {
	const double dt = t - mT;
	return MotionState(t, (mPos + mVel * dt + 0.5 * acc * dt), (mVel + acc * dt), acc);
}

double MotionState::nextTimeAtPos(double pos) const {
	if (epsilonEquals(pos, mPos, kEpsilon)) {
		// Already at pos.
		return mT;
	}
	if (epsilonEquals(mAcc, 0.0, kEpsilon)) {
		// Zero acceleration case.
		const double deltaPos = pos - mPos;
		if (!epsilonEquals(mVel, 0.0, kEpsilon) && signum(deltaPos) == signum(mVel)) {
			// Constant velocity heading towards pos.
			return mT + deltaPos / mVel;
		}
		return kNaN;
	}
	// Solve the quadratic formula.
	// ax^2 + bx + c == 0
	// x = dt
	// a = 0.5 * acc
	// b = vel
	// c = this->pos - pos
	const double disc = mVel * mVel - 2.0 * mAcc * (mPos - pos);
	if (disc < 0.0) {
		// Extrapolating this MotionState never reaches the desired pos.
		return kNaN;
	}
	const double sqrtDisc = std::sqrt(disc);
	const double maxDt = (-mVel + sqrtDisc) / mAcc;
	const double minDt = (-mVel - sqrtDisc) / mAcc;
	if (minDt >= 0.0 && (maxDt < 0.0 || minDt < maxDt)) {
		return mT + minDt;
	}
	if (maxDt >= 0.0) {
		return mT + maxDt;
	}
	// We only reach the desired pos in the past.
	return kNaN;
}

bool MotionState::isValid() const {
	return !std::isnan(mT) || !std::isnan(mPos) || !std::isnan(mVel) || !std::isnan(mAcc);
}

bool MotionState::equals(MotionState other) const {
	return equals(other, kEpsilon);
}

bool MotionState::equals(MotionState other, double epsilon) const {
	bool bothInvalid = std::isnan(mT) && std::isnan(other.mT) && std::isnan(mPos)
			&& std::isnan(other.mPos) && std::isnan(mVel) && std::isnan(other.mVel)
			&& std::isnan(mAcc) && std::isnan(other.mAcc);
	return bothInvalid || (coincident(other, epsilon) && epsilonEquals(mAcc, other.mAcc, epsilon));
}

bool MotionState::coincident(MotionState other) const {
	return coincident(other, kEpsilon);
}

bool MotionState::coincident(MotionState other, double epsilon) const {
	return epsilonEquals(mT, other.mT, epsilon) && epsilonEquals(mPos, other.mPos, epsilon)
			&& epsilonEquals(mVel, other.mVel, epsilon);
}

MotionState MotionState::flipped() const {
	return MotionState(mT, -mPos, -mVel, -mAcc);
}

}  // namespace bns
