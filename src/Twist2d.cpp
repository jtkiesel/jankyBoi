#include "Twist2d.hpp"

namespace bns {

const Twist2d Twist2d::kIdentity = Twist2d();

const Twist2d Twist2d::identity() {
	return kIdentity;
}

Twist2d::Twist2d() : Twist2d(0.0, 0.0, 0.0) {}

Twist2d::Twist2d(double dx, double dy, double dtheta) : kDx(dx), kDy(dy), kDtheta(dtheta) {}

Twist2d Twist2d::scaled(double s) const {
	return Twist2d(kDx * s, kDy * s, kDtheta * s);
}

}  // namespace bns
