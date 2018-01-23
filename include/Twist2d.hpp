#ifndef TWIST2D_HPP_
#define TWIST2D_HPP_

namespace bns {

/**
 * A movement along an arc at constant curvature and velocity. We can use ideas from "differential calculus" to create
 * new RigidTransform2d's from a Twist2d and vice versa.
 *
 * A Twist can be used to represent a difference between two poses, a velocity, an acceleration, etc.
 */
class Twist2d {
public:
	static const Twist2d identity();
	const double kDx;
	const double kDy;
	const double kDtheta;
	Twist2d();
	Twist2d(double dx, double dy, double dtheta);
	Twist2d scaled(double s) const;
protected:
	static const Twist2d kIdentity;
};

}  // namespace bns

#endif  // TWIST2D_HPP_
