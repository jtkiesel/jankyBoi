#ifndef RIGIDTRANSFORM2D_HPP_
#define RIGIDTRANSFORM2D_HPP_

#include "Interpolable.hpp"
#include "Rotation2d.hpp"
#include "Translation2d.hpp"
#include "Twist2d.hpp"

namespace bns {

/**
 * Represents a 2D pose (rigid transform) containing translational and
 * rotational elements.
 *
 * Inspired by Sophus (https://github.com/strasdat/Sophus/tree/master/sophus).
 */
class RigidTransform2d {
public:
	RigidTransform2d();
	RigidTransform2d(Translation2d translation, Rotation2d rotation);
	RigidTransform2d(const RigidTransform2d &other);
	static RigidTransform2d fromTranslation(Translation2d translation);
	static RigidTransform2d fromRotation(Rotation2d rotation);
	/**
	 * Obtain a new RigidTransform2d from a (constant curvature) velocity. See:
	 * https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp
	 */
	static RigidTransform2d exp(Twist2d delta);
	/**
	 * Logical inverse of exp.
	 */
	static Twist2d log(RigidTransform2d transform);
	Translation2d translation() const;
	void setTranslation(Translation2d translation);
	Rotation2d rotation() const;
	void setRotation(Rotation2d rotation);
	/**
	 * Transforming this RigidTransform2d means first translating by
	 * other.translation() and then rotating by other.rotation().
	 *
	 * @param other  The other transform.
	 * @return This transform * other.
	 */
	RigidTransform2d transformBy(RigidTransform2d other) const;
	/**
	 * The inverse of this transform "undoes" the effect of translating by this
	 * transform.
	 *
	 * @return The opposite of this transform.
	 */
	RigidTransform2d inverse() const;
	RigidTransform2d normal() const;
	/**
	 * Finds the point where the heading of this transform intersects the
	 * heading of another. Returns (+INFINITY, +INFINITY) if parallel.
	 */
	Translation2d intersection(RigidTransform2d other) const;
	/**
	 * Return true if the heading of this transform is colinear with the heading
	 * of another.
	 */
	bool isColinear(RigidTransform2d other) const;
	/**
	 * Do twist interpolation of this transform assuming constant curvature.
	 */
	RigidTransform2d interpolate(RigidTransform2d other, double x) const;
protected:
	static constexpr double kEpsilon = 1E-9;
	static const RigidTransform2d kIdentity;
	static const RigidTransform2d identity();
	Translation2d mTranslation;
	Rotation2d mRotation;
private:
	static Translation2d intersection(RigidTransform2d a, RigidTransform2d b);
};

}

#endif  // RIGIDTRANSFORM2D_HPP_
