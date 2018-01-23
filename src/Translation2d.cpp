#include "Translation2d.hpp"

#include "cpp.hpp"
#include "Rotation2d.hpp"

#include <algorithm>
#include <cmath>

namespace bns {

const Translation2d Translation2d::kIdentity = Translation2d();

const Translation2d Translation2d::identity() {
	return kIdentity;
}

Translation2d::Translation2d() : Translation2d(0.0, 0.0) {}

Translation2d::Translation2d(double x, double y) : mX(x), mY(y) {}

Translation2d::Translation2d(const Translation2d &other) : Translation2d(other.mX, other.mY) {}

Translation2d::Translation2d(Translation2d start, Translation2d end) : Translation2d(end.mX - start.mX, end.mY - start.mY) {}

double Translation2d::norm() const {
	return std::hypot(mX, mY);
}

double Translation2d::x() const {
	return mX;
}

void Translation2d::setX(double x) {
	mX = x;
}

double Translation2d::y() const {
	return mY;
}

void Translation2d::setY(double y) {
 	mY = y;
}

Translation2d Translation2d::translateBy(Translation2d other) const {
	return Translation2d(mX + other.mX, mY + other.mY);
}

Translation2d Translation2d::rotateBy(Rotation2d rotation) const {
	return Translation2d(mX * rotation.cos() - mY * rotation.sin(), mX * rotation.sin() + mY * rotation.cos());
}

Rotation2d Translation2d::direction() const {
	return Rotation2d(mX, mY, true);
}

Translation2d Translation2d::inverse() const {
	return Translation2d(-mX, -mY);
}

Translation2d Translation2d::interpolate(Translation2d other, double x) const {
	if (x <= 0) {
		return Translation2d(*this);
	} else if (x >= 1) {
		return Translation2d(other);
	}
	return extrapolate(other, x);
}

Translation2d Translation2d::extrapolate(Translation2d other, double x) const {
	return Translation2d(x * (other.mX - mX) + mX, x * (other.mY - mY) + mY);
}

Translation2d Translation2d::scale(double s) const {
	return Translation2d(mX * s, mY * s);
}

double Translation2d::dot(Translation2d a, Translation2d b) {
	return a.mX * b.mX + a.mY * b.mY;
}

Rotation2d Translation2d::rotation(Translation2d a, Translation2d b) {
	const double cosAngle = dot(a, b) / (a.norm() * b.norm());
	if (std::isnan(cosAngle)) {
		return Rotation2d();
	}
	return Rotation2d::fromRadians(std::acos(std::min(1.0, std::max(cosAngle, -1.0))));
}

double Translation2d::cross(Translation2d a, Translation2d b) {
	return a.mX * b.mY - a.mY * b.mX;
}

}  // namespace bns
