#include "util.h"

#include "API.h"

#include <math.h>

const double kPi = 3.14159265358979323846;

double signum(double d) {
	return (d == 0.0 || isnan(d)) ? d : copysign(1.0, d);
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
	while (radians < -kPi) {
		radians += 2 * kPi;
	}
	while (radians >= kPi) {
		radians -= 2 * kPi;
	}
	return radians;
}

double clamp(double value, double min, double max) {
	double temp = (value < min) ? min : value;
	return (temp > max) ? max : temp;
}

double clampAbs(double value, double maxAbs) {
	return clamp(value, -maxAbs, maxAbs);
}

int fgetw(PROS_FILE *stream) {
	int c1 = fgetc(stream);
	int c2 = fgetc(stream);

	if (c1 < 0 || c2 < 0) {
		return -1;
	}
	return c1 | (c2 << 8);
}
