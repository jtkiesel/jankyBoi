#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

typedef struct PidController {
	double Kp;
	double Ki;
	double Kd;
	double error;
	unsigned long t;
	double integral;
	double output;
} PidController;

PidController pidControllerCreate(double Kp, double Ki, double Kd);

double pidControllerComputeOutput(PidController* pidController, double error, unsigned long t);

double pidControllerOutput(const PidController* pidController);

#endif  // PIDCONTROLLER_H_
