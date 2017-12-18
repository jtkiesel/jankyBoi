/** @file opcontrol.cpp
 * @brief File for operator control code
 *
 * This file should contain the user operatorControl() function and any functions related to it.
 *
 * Any copyright is dedicated to the Public Domain.
 * http://creativecommons.org/publicdomain/zero/1.0/
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.hpp"

#include <cmath>

#include "constants.hpp"
#include "TrapezoidalMotionProfile.hpp"
#include "PidController.hpp"

using namespace std;
using namespace bns;

/**
 * Adjusts a desired power based on the current battery voltage.
 *
 * @param power  Desired power.
 * @param volts  Current battery voltage.
 */
double batteryAdjustedPower(double power, double volts) {
	return power / (0.1845977782 * volts - 0.08777418571);
}

/**
 * Converts from output power to PWM value, in order to linearize motor output.
 *
 * @param power  Desired percentage of max power, between -1 and 1.
 * @return       PWM value that will come closest to achieving the desired power.
 */
int powerToPwm(double power) {
	if (power == 0) {
		return 0;
	}
	double p = std::abs(power);
	if (p >= 1) {
		return std::copysign(127, power);
	}
	return std::copysign(((((((((-44128.10541 * p + 178572.6802) * p - 297071.4563) * p
			+ 262520.7547) * p - 132692.6561) * p + 38464.48054) * p - 6049.717501) * p
			+ 476.2279947) * p - 1.233957961), power);
}

void motorSetLinear(unsigned char channel, double power) {
	motorSet(channel, powerToPwm(power));
}

void motorSetAtVolts(unsigned char channel, double power, double volts) {
	motorSetLinear(channel, batteryAdjustedPower(power, volts));
}

/*
 * Runs the user operator control code. This function will be started in its own task with the
 * default priority and stack size whenever the robot is enabled via the Field Management System
 * or the VEX Competition Switch in the operator control mode. If the robot is disabled or
 * communications is lost, the operator control task will be stopped by the kernel. Re-enabling
 * the robot will restart the task, not resume it from where it left off.
 *
 * If no VEX Competition Switch or Field Management system is plugged in, the VEX Cortex will
 * run the operator control task. Be warned that this will also occur if the VEX Cortex is
 * tethered directly to a computer via the USB A to A cable without any VEX Joystick attached.
 *
 * Code running in this task can take almost any action, as the VEX Joystick is available and
 * the scheduler is operational. However, proper use of delay() or taskDelayUntil() is highly
 * recommended to give other tasks (including system tasks such as updating LCDs) time to run.
 *
 * This task should never exit; it should end with some kind of infinite loop, even if empty.
 */
void operatorControl() {
	printf("starting\n");
	delay(3000);

	const double vMax = 1.2;
	const double aMax = 0.001;
	const double KvFf = 1 / vMax;
	const double KaFf = 150;
	const double Kv = 1.5;
	const double Kp = 0.01;
	const double Ki = 0;
	const double Kd = 0.01;
	const unsigned long t0 = millis();
	int x0;
	imeReset(DRIVE_IME_LEFT);
	imeGet(DRIVE_IME_LEFT, &x0);
	TrapezoidalMotionProfile motionProfile(vMax, aMax, x0, 1000);

	PidController pidController(Kp, Ki, Kd);
	unsigned long t = 0;
	int x;
	double xError;
	double v;
	double vError;
	int xLast = x0;
	unsigned long tLast = t0;
	double power;
	MotionProfile::Snapshot setpoint;

	while (!motionProfile.isDone(t)) {
		imeGet(DRIVE_IME_LEFT, &x);
		setpoint = motionProfile.computeSnapshot(x, t);
		v = (t - tLast) > 0 ? ((double)(x - xLast) / (t - tLast)) : setpoint.v;

		xError = setpoint.x - x;
		vError = setpoint.v - v;

		power = KvFf * setpoint.v + KaFf * setpoint.a + Kv * vError + pidController.computeOutput(xError, t);
		motorSetLinear(DRIVE_LEFT, power);

printf("%f,", v);
		motionProfile.graph(x);

		delay(20);
		xLast = x;
		tLast = t;
		t = millis() - t0;
	}
}
