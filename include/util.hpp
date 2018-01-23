#ifndef UTIL_HPP_
#define UTIL_HPP_

#include <limits>

namespace bns {

constexpr double kPi = 3.14159265358979323846264338327950288;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

constexpr double kInfinity = std::numeric_limits<double>::infinity();

/**
 * Returns the signum function of the argument; zero if the argument is zero,
 * 1.0 if the argument is greater than zero, -1.0 if the argument is less than
 * zero.
 *
 * Special Cases:
 * <ul>
 * <li> If the argument is NaN, then the result is NaN.
 * <li> If the argument is positive zero or negative zero, then the result is
 * the same as the argument.
 * </ul>
 *
 * @param d  The floating-point value whose signum is to be returned.
 * @return   The signum function of the argument.
 */
double signum(double d);

double toDegrees(double angleRadians);

double toRadians(double angleDegrees);

bool epsilonEquals(double a, double b, double epsilon);

}  // namespace bns

#endif  // UTIL_HPP_
