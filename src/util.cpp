#include "util.hpp"

#include <cmath>

namespace bns {

double signum(double d) {
	return (d == 0.0 || std::isnan(d)) ? d : std::copysign(1.0, d);
}

double toDegrees(double radians) {
	return radians * 180.0 / kPi;
}

double toRadians(double degrees) {
	return degrees * kPi / 180.0;
}

bool epsilonEquals(double a, double b, double epsilon) {
	return (a - epsilon <= b) && (a + epsilon >= b);
}

}  // namespace bns
