#ifndef UTIL_H_
#define UTIL_H_

#include "API.h"

extern const double kPi;

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

double toDegrees(double radians);

double toRadians(double degrees);

double boundAngle0To2Pi(double radians);

double boundAngleNegPiToPi(double radians);

/**
 * Get next word from stream.
 *
 * @param  stream to get from.
 * @return Next word, or -1 if 2 characters cannot be read.
 */
int fgetw(PROS_FILE *stream);

#endif  // UTIL_H_
