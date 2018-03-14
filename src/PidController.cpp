#include "PidController.hpp"

#include <climits>

namespace bns {

PidController::PidController(double Kp, double Ki, double Kd) : kP(Kp), kI(Ki), kD(Kd), mError(0),
		mT(ULONG_MAX), mIntegral(0), mOutput(0) {}

double PidController::computeOutput(double error, unsigned long t) {
	const double dt = (mT == ULONG_MAX) ? 0 : (t - mT);
	mIntegral += error * dt;
	const double derivative = (dt == 0) ? 0 : ((error - mError) / dt);

	mOutput = kP * error + kI * mIntegral + kD * derivative;
	mT = t;

	return mOutput;
}

double PidController::output() const {
	return mOutput;
}

}  // namespace bns
