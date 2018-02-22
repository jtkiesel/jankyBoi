#include "Twist2d.hpp"

namespace bns {

const Twist2d Twist2d::kIdentity = Twist2d();

const Twist2d Twist2d::identity() {
	return kIdentity;
}

Twist2d::Twist2d() : Twist2d(0.0, 0.0, 0.0) {}

Twist2d::Twist2d(double dx, double dy, double dtheta) : mDx(dx), mDy(dy), mDtheta(dtheta) {}

double Twist2d::dx() const {
	return mDx;
}

double Twist2d::dy() const {
	return mDy;
}

double Twist2d::dtheta() const {
	return mDtheta;
}

Twist2d Twist2d::scaled(double s) const {
	return Twist2d(mDx * s, mDy * s, mDtheta * s);
}

}  // namespace bns
