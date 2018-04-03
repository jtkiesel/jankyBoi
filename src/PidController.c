#include "PidController.h"

#include "API.h"

#include <limits.h>

PidController pidControllerCreate(double Kp, double Ki, double Kd) {
	return (PidController) {.Kp = Kp, .Ki = Ki, .Kd = Kd, .error = 0.0, .t = ULONG_MAX,
			.integral = 0.0, .output = 0.0};
}

double pidControllerComputeOutput(PidController* pidController, double error, unsigned long t) {
	if (!pidController) {
		printf("Error - pidControllerComputeOutput: pidController NULL.\n");
		return 0.0;
	}
	const unsigned long dt = (pidController->t == ULONG_MAX) ? 0 : (t - pidController->t);
	pidController->integral += error * dt;
	const double derivative = (dt == 0) ? 0.0 : ((error - pidController->error) / dt);

	pidController->output = pidController->Kp * error + pidController->Ki * pidController->integral
			+ pidController->Kd * derivative;
	pidController->t = t;
	pidController->error = error;

	return pidController->output;
}

double pidControllerOutput(const PidController* pidController) {
	if (!pidController) {
		printf("Error - pidControllerOutput: pidController NULL.\n");
		return 0.0;
	}
	return pidController->output;	
}
