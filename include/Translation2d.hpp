#ifndef TRANSLATION2D_HPP_
#define TRANSLATION2D_HPP_

#include "Interpolable.hpp"
#include "Rotation2d.hpp"

namespace bns {

class Rotation2d;

/**
 * A translation in a 2d coordinate frame. Translations are simply shifts in an (x, y) plane.
 */
class Translation2d : Interpolable<Translation2d> {
public:
	static const Translation2d identity();
	Translation2d();
	Translation2d(double x, double y);
	Translation2d(const Translation2d &other);
	Translation2d(Translation2d start, Translation2d end);
	/**
	 * The "norm" of a transform is the Euclidean distance in x and y.
	 *
	 * @return  sqrt(x^2 + y^2)
	 */
	double norm() const;
	double norm2() const;
	double x() const;
	void setX(double x);
	double y() const;
	void setY(double y);
	/**
	 * We can compose Translation2d's by adding together the x and y shifts.
	 *
	 * @param other  The other translation to add.
	 * @return       The combined effect of translating by this object and the other.
	 */
	Translation2d translateBy(Translation2d other) const;
	/**
	 * We can also rotate Translation2d's. See: https://en.wikipedia.org/wiki/Rotation_matrix
	 *
	 * @param rotation  The rotation to apply.
	 * @return          This translation rotated by rotation.
	 */
	Translation2d rotateBy(Rotation2d rotation) const;
	Rotation2d direction() const;
	/**
	 * The inverse simply means a Translation2d that "undoes" this object.
	 *
	 * @return Translation by -x and -y.
	 */
	Translation2d inverse() const;
	Translation2d interpolate(Translation2d other, double x) const;
	Translation2d extrapolate(Translation2d other, double x) const;
	Translation2d scale(double s) const;
	static double dot(Translation2d a, Translation2d b);
	static Rotation2d rotation(Translation2d a, Translation2d b);
	static double cross(Translation2d a, Translation2d b);
protected:
	static const Translation2d kIdentity;
	double mX;
	double mY;
};

}  // namespace bns

#endif  // TRANSLATION2D_HPP_
