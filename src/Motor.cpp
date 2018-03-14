#include "Motor.hpp"

#include "api.hpp"

#include <cmath>

namespace bns {

Motor::Motor(unsigned char port, bool reversed) : port(port), direction(reversed ? -1 : 1) {
	if (port < 1 || port > 10) {
		pros::printf("Error: Motor port out of range.");
	}
}

void Motor::setPwm(int pwm) const {
	pros::motorSet(port, (direction * pwm));
}

void Motor::setPower(double power) const {
	setPwm(powerToPwm(power));
}

int Motor::powerToPwm(double power) {
	if (power == 0.0) {
		return 0;
	}
	double p = std::abs(power);
	if (p >= 1.0) {
		return static_cast<int>(std::round(std::copysign(127.0, power)));
	}
	return static_cast<int>(std::round(std::copysign(((((((((-44128.10541 * p + 178572.6802) * p
			- 297071.4563) * p + 262520.7547) * p - 132692.6561) * p + 38464.48054) * p
			- 6049.717501) * p + 476.2279947) * p - 1.233957961), power)));
}

}  // namespace bns
