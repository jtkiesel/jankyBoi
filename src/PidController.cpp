#include "PidController.hpp"

namespace bns {

PidController::PidController(double Kp, double Ki, double Kd) : Kp(Kp), Ki(Ki), Kd(Kd) {
	error = 0;
	integral = 0;
	output = 0;
}

double PidController::computeOutput(double error, unsigned long t) {
	const double dt = t - this->t;
	integral += error * dt;
	const double derivative = (dt == 0) ? 0 : ((error - this->error) / dt);

	output = Kp * error + Ki * integral + Kd * derivative;

	return output;
}

double PidController::getOutput() const {
	return output;
}

};  // namespace bns
