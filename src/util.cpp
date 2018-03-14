#include "util.hpp"
#include "api.hpp"
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

double boundAngle0To2Pi(double radians) {
	while (radians < 0) {
		radians += 2 * kPi;
	}
	while (radians >= 2 * kPi) {
		radians -= 2 * kPi;
	}
	return radians;
}

double boundAngleNegPiToPi(double radians) {
pros::printf("13\n");
	while (radians < -kPi) {
		radians += 2 * kPi;
	}
pros::printf("14\n");
	while (radians >= kPi) {
		radians -= 2 * kPi;
	}
pros::printf("15\n");
	return radians;
}

bool epsilonEquals(double a, double b, double epsilon) {
	return (a - epsilon <= b) && (a + epsilon >= b);
}

}  // namespace bns
