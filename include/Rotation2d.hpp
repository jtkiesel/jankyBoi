#ifndef ROTATION2D_HPP_
#define ROTATION2D_HPP_

#include "Interpolable.hpp"
#include "Translation2d.hpp"

namespace bns {

class Translation2d;

/**
 * A rotation in a 2d coordinate frame represented a point on the unit circle (cosine and sine).
 *
 * Inspired by Sophus (https://github.com/strasdat/Sophus/tree/master/sophus)
 */
class Rotation2d : Interpolable<Rotation2d> {
public:
	static const Rotation2d identity();
	Rotation2d();
	Rotation2d(double x, double y, bool normalize);
	Rotation2d(const Rotation2d &other);
	Rotation2d(Translation2d direction, bool normalize);
	static Rotation2d fromRadians(double angleRadians);
	static Rotation2d fromDegrees(double angleDegrees);
	/**
	 * From trig, we know that sin^2 + cos^2 == 1, but as we do math on this object we might accumulate rounding errors.
	 * Normalizing forces us to re-scale the sin and cos to reset rounding errors.
	 */
	void normalize();
	double cos() const;
	double sin() const;
	double tan() const;
	double radians() const;
	double degrees() const;
	/**
	 * We can rotate this Rotation2d by adding together the effects of it and another rotation.
	 *
	 * @param other  The other rotation. See: https://en.wikipedia.org/wiki/Rotation_matrix
	 * @return       This rotation rotated by other.
	 */
	Rotation2d rotateBy(Rotation2d other) const;
	Rotation2d normal() const;
	/**
	 * The inverse of a Rotation2d "undoes" the effect of this rotation.
	 *
	 * @return  The opposite of this rotation.
	 */
	Rotation2d inverse() const;
	bool isParallel(Rotation2d other) const;
	Translation2d toTranslation() const;
	Rotation2d interpolate(Rotation2d other, double x) const;
protected:
	static const Rotation2d kIdentity;
	static const double kEpsilon;
	double mCos;
	double mSin;
};

}  // namespace bns

#endif  // TRANSLATION2D_HPP_
