#include "PidController.hpp"

#include <climits>

namespace bns {

PidController::PidController(double Kp, double Ki, double Kd) : Kp(Kp), Ki(Ki), Kd(Kd) {
	error = 0;
	t = ULONG_MAX;
	integral = 0;
	output = 0;
}

double PidController::computeOutput(double error, unsigned long t) {
	const double dt = (this->t == ULONG_MAX) ? 0 : (t - this->t);
	integral += error * dt;
	const double derivative = (dt == 0) ? 0 : ((error - this->error) / dt);

	output = Kp * error + Ki * integral + Kd * derivative;
	this->t = t;

	return output;
}

double PidController::getOutput() const {
	return output;
}

};  // namespace bns
