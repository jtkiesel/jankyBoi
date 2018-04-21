#include "Motor.h"

#include "API.h"
#include "log.h"

#include <math.h>
#include <stdbool.h>

Motor motorCreate(unsigned char port, bool reversed) {
	if (port > 10) {
		logError("motorCreate", "port > 10");
		return (Motor) {};
	}
	return (Motor) {.port = port, .direction = (signed char) (reversed ? -1 : 1)};
}

void motorSetPwm(const Motor* motor, int pwm) {
	if (!motor) {
		logError("motorSetPwm", "motor NULL");
		return;
	}
	motorSet(motor->port, (motor->direction * pwm));
}

void motorSetPower(const Motor* motor, double power) {
	if (!motor) {
		logError("motorSetPower", "motor NULL");
		return;
	}
	motorSetPwm(motor, powerToPwm(power));
}

int powerToPwm(double power) {
	/*double p = fabs(power);
	if (p < 0.000001) {
		return 0;
	}
	if (p > 0.999999) {
		return (int) copysign(127.0, power);
	}
	return (int) round(copysign(((((((((-44128.10541 * p + 178572.6802) * p
			- 297071.4563) * p + 262520.7547) * p - 132692.6561) * p + 38464.48054) * p
			- 6049.717501) * p + 476.2279947) * p - 1.233957961), power));*/
	double p = fabs(power);
	if (p < 0.000001) {
		return 0;
	}
	if (p > 0.999999) {
		return (int) copysign(127.0, power);
	}
	return (int) round(copysign((82.91 * power + 5.0854), power));
}
