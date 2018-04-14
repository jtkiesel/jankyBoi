#include "PidController.h"

#include "API.h"
#include "log.h"

#include <limits.h>
#include <math.h>

PidController pidControllerCreate(double Kp, double Ki, double Kd) {
	return (PidController) {.Kp = Kp, .Ki = Ki, .Kd = Kd, .error = 0.0, .t = ULONG_MAX,
			.integral = 0.0, .output = 0.0};
}

double pidControllerComputeOutput(PidController* pidController, double error, unsigned long t) {
	if (!pidController) {
		logError("pidControllerComputeOutput", "pidController NULL");
		return NAN;
	}
	const unsigned long dt = (pidController->t == ULONG_MAX) ? 0 : (t - pidController->t);
	pidController->integral += error * dt;
	const double derivative = (dt == 0) ? 0.0 : ((error - pidController->error) / dt);
	printf("derivative: %f\n", pidController->Kd * derivative);

	pidController->output = pidController->Kp * error + pidController->Ki * pidController->integral
			+ pidController->Kd * derivative;
	pidController->t = t;
	pidController->error = error;

	return pidController->output;
}

double pidControllerOutput(const PidController* pidController) {
	if (!pidController) {
		logError("pidControllerOutput", "pidController NULL");
		return 0.0;
	}
	return pidController->output;
}
