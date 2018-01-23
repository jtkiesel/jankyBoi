#include "Rotation2d.hpp"

#include "cpp.hpp"
#include "Translation2d.hpp"
#include "util.hpp"

#include <cmath>
#include <limits>

namespace bns {

const Rotation2d Rotation2d::kIdentity = Rotation2d();

const Rotation2d Rotation2d::identity() {
	return kIdentity;
}

const double Rotation2d::kEpsilon = 1E-9;

Rotation2d::Rotation2d() : Rotation2d(1, 0, false) {}

Rotation2d::Rotation2d(double x, double y, bool normalize) : mCos(x), mSin(y) {
	if (normalize) {
		this->normalize();
	}
}

Rotation2d::Rotation2d(const Rotation2d &other) : Rotation2d(other.mCos, other.mSin, false) {}

Rotation2d::Rotation2d(Translation2d direction, bool normalize) : Rotation2d(direction.x(), direction.y(), normalize) {}

Rotation2d Rotation2d::fromRadians(double radians) {
	return Rotation2d(std::cos(radians), std::sin(radians), false);
}

Rotation2d Rotation2d::fromDegrees(double degrees) {
	return fromRadians(toRadians(degrees));
}

void Rotation2d::normalize() {
	double magnitude = std::hypot(mCos, mSin);
	if (magnitude > kEpsilon) {
		mCos /= magnitude;
		mSin /= magnitude;
	} else {
		mCos = 1;
		mSin = 0;
	}
}

double Rotation2d::cos() const {
	return mCos;
}

double Rotation2d::sin() const {
	return mSin;
}

double Rotation2d::tan() const {
	if (std::abs(mCos) < kEpsilon) {
		if (mSin >= 0.0) {
			return std::numeric_limits<double>::infinity();
		} else {
			return -std::numeric_limits<double>::infinity();
		}
	}
	return mSin / mCos;
}

double Rotation2d::radians() const {
	return std::atan2(mSin, mCos);
}

double Rotation2d::degrees() const {
	return toDegrees(radians());
}

Rotation2d Rotation2d::rotateBy(Rotation2d other) const {
	return Rotation2d(mCos * other.mCos - mSin * other.mSin, mCos * other.mSin + mSin * other.mCos, true);
}

Rotation2d Rotation2d::normal() const {
	return Rotation2d(-mSin, mCos, false);
}

Rotation2d Rotation2d::inverse() const {
	return Rotation2d(mCos, -mSin, false);
}

bool Rotation2d::isParallel(Rotation2d other) const {
	return epsilonEquals(Translation2d::cross(toTranslation(), other.toTranslation()), 0.0, kEpsilon);
}

Translation2d Rotation2d::toTranslation() const {
	return Translation2d(mCos, mSin);
}

Rotation2d Rotation2d::interpolate(Rotation2d other, double x) const {
	if (x <= 0) {
		return Rotation2d(*this);
	} else if (x >= 1) {
		return Rotation2d(other);
	}
	double angleDiff = inverse().rotateBy(other).radians();
	return rotateBy(fromRadians(angleDiff * x));
}

}  // namespace bns
