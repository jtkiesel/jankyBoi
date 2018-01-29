#include "RigidTransform2d.hpp"

#include "Rotation2d.hpp"
#include "Translation2d.hpp"
#include "Twist2d.hpp"
#include "util.hpp"

#include <cmath>

namespace bns {

const RigidTransform2d RigidTransform2d::kIdentity = RigidTransform2d();

const RigidTransform2d RigidTransform2d::identity() {
	return kIdentity;
}

RigidTransform2d::RigidTransform2d() {}

RigidTransform2d::RigidTransform2d(Translation2d translation, Rotation2d rotation) :
		mTranslation(translation), mRotation(rotation) {}

RigidTransform2d::RigidTransform2d(const RigidTransform2d &other) :
		mTranslation(other.mTranslation), mRotation(other.mRotation) {}

RigidTransform2d RigidTransform2d::exp(Twist2d delta) {
	double sinTheta = std::sin(delta.dtheta());
	double cosTheta = std::cos(delta.dtheta());
	double s, c;
	if (std::abs(delta.dtheta()) < kEpsilon) {
		s = 1.0 - 1.0 / 6.0 * delta.dtheta() * delta.dtheta();
		c = 0.5 * delta.dtheta();
	} else {
		s = sinTheta / delta.dtheta();
		c = (1.0 - cosTheta) / delta.dtheta();
	}
	return RigidTransform2d(Translation2d(delta.dx() * s - delta.dy() * c,
			delta.dx() * c + delta.dy() * s), Rotation2d(cosTheta, sinTheta, false));
}

Twist2d RigidTransform2d::log(RigidTransform2d transform) {
	const double dtheta = transform.mRotation.radians();
	const double halfDtheta = 0.5 * dtheta;
	const double cosMinus1 = transform.mRotation.cos() - 1.0;
	double halfThetaByTanHalfTheta;
	if (std::abs(cosMinus1) < kEpsilon) {
		halfThetaByTanHalfTheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
	} else  {
		halfThetaByTanHalfTheta = -(halfDtheta * transform.mRotation.sin()) / cosMinus1;
	}
	const Translation2d translation = transform.translation()
			.rotateBy(Rotation2d(halfThetaByTanHalfTheta, -halfDtheta, false));
	return Twist2d(translation.x(), translation.y(), dtheta);
}

Translation2d RigidTransform2d::translation() const {
	return mTranslation;
}

void RigidTransform2d::setTranslation(Translation2d translation) {
	mTranslation = translation;
}

Rotation2d RigidTransform2d::rotation() const {
	return mRotation;
}

void RigidTransform2d::setRotation(Rotation2d rotation) {
	mRotation = rotation;
}

RigidTransform2d RigidTransform2d::transformBy(RigidTransform2d other) const {
	return RigidTransform2d(mTranslation.translateBy(other.mTranslation.rotateBy(mRotation)),
			mRotation.rotateBy(other.mRotation));
}

RigidTransform2d RigidTransform2d::inverse() const {
	Rotation2d rotation = mRotation.inverse();
	return RigidTransform2d(mTranslation.inverse().rotateBy(rotation), rotation);
}

RigidTransform2d RigidTransform2d::normal() const {
	return RigidTransform2d(mTranslation, mRotation.normal());
}

Translation2d RigidTransform2d::intersection(RigidTransform2d other) const {
	if (mRotation.isParallel(other.mRotation)) {
		// Lines are parallel.
		return Translation2d(kInfinity, kInfinity);
	}
	if (std::abs(mRotation.cos()) < std::abs(other.mRotation.cos())) {
		return intersection(*this, other);
	}
	return intersection(other, *this);
}

bool RigidTransform2d::isColinear(RigidTransform2d other) const {
	const Twist2d twist = log(inverse().transformBy(other));
	return epsilonEquals(twist.dy(), 0.0, kEpsilon) && epsilonEquals(twist.dtheta(), 0.0, kEpsilon);
}

Translation2d RigidTransform2d::intersection(RigidTransform2d a, RigidTransform2d b) {
	const Rotation2d aRotation = a.rotation();
	const Rotation2d bRotation = b.rotation();
	const Translation2d aTranslation = a.translation();
	const Translation2d bTranslation = b.translation();

	const double bTan = bRotation.tan();
	const double t = ((aTranslation.x() - bTranslation.x()) * bTan + bTranslation.y() - aTranslation.y())
			/ (aRotation.sin() - aRotation.cos() * bTan);
	return aTranslation.translateBy(aRotation.toTranslation().scale(t));
}

RigidTransform2d RigidTransform2d::interpolate(RigidTransform2d other, double x) const {
	if (x <= 0.0) {
		return *this;
	}
	if (x >= 1.0) {
		return other;
	}
	return transformBy(exp(log(inverse().transformBy(other)).scaled(x)));
}

}  // namespace bns
