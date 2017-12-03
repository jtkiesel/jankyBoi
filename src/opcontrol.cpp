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

#include <math.h>

#include "bnsMath.hpp"
#include "TrapezoidalMotionProfile.hpp"
#include "PidController.hpp"

using namespace std;
using namespace bns;

/**
 * Adjusts a target velocity based on the current battery voltage. Voltage is
 * passed in as a percentage of the "specified velocity" (100 RPM, 160 RPM, or
 * 240 RPM), and is returned as a percentage of the max velocity possible at the
 * passed in battery voltage.
 */
double batteryAdjustedVelocityPct(double velocityPct, double volts) {
	return velocityPct / (0.1845977782 * volts - 0.08777418571);
}

int velocityPctToMotorSpeed(double velocityPct) {
	if (velocityPct == 0) {
		return 0;
	}
	double v = fabs(velocityPct);
	if (v > 0.99) {
		return sgn(velocityPct) * 127;
	}
	return sgn(velocityPct) * ((((((((((-512553.361 * v + 2536180.47) * v - 5371392.274) * v
			+ 6365170.363) * v - 4631143.823) * v + 2137181.669) * v - 624465.95) * v
			+ 112018.51) * v - 11522.344) * v + 615.694) * v - 2.342);
}

enum MotorVelocity {
	MotorTorque = 100,
	MotorSpeed = 160,
	MotorTurbo = 240
};

void motorSetVelocityAtVolts(unsigned char channel, double velocity, double volts) {
	//printf("motorSet at v: %f\n", velocity);
	motorSet(channel, velocityPctToMotorSpeed(batteryAdjustedVelocityPct(velocity, volts)));
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
	delay(5000);

	print("starting\n");

	const double maxVel = 1.80;
	const double Kv = 100 / maxVel;
	const double Ka = 10;
	const double Kp = 0;
	const double Ki = 0;
	const double Kd = 0;
	const unsigned long t0 = millis();
	const int x0 = analogRead(BOTTOM_POT);
	const TrapezoidalMotionProfile motionProfile(1.5, 0.01, x0, 1000);

	PidController pidController(Kp, Ki, Kd);
	unsigned long t;
	int x;
	double error;
	int speed;
	MotionProfile::Snapshot setpoint;

	while (true) {
		t = millis() - t0;
		x = analogRead(BOTTOM_POT);

		setpoint = motionProfile.getSnapshot(t);
		error = setpoint.x - x;

		speed = Kv * setpoint.v + Ka * setpoint.a + pidController.computeOutput(error, t);
		motorSet(2, velocityPctToMotorSpeed(speed));

		delay(20);
	}
/*
	TrapezoidalMotionProfile motionProfile(100, 1, 1000, 0, 0);

	const double maxVelocity = 100;
	const double Kv = 1 / maxVelocity;
	const double Ka = 1.0;
	const double Kp = 1.0;
	const double Kd = 0;
	const unsigned long t0 = millis();
	const int x0 = encoderGet(encoder);

	int counts;
	int error;
	int lastError;
	double derivative;
	double velocity;
	unsigned long t;
	unsigned long dt;
	unsigned long lastT = 0;
	MotionProfile::Snapshot setpoint;

	while (true) {
		counts = encoderGet(encoder) - x0;
		t = millis() - t0;

		setpoint = motionProfile.getSnapshot(t);
		error = setpoint.x - counts;
		dt = t - lastT;
		derivative = (dt == 0) ? 0 : ((error - lastError) / dt - setpoint.v);

		velocity = Kv * setpoint.v + Ka * setpoint.a + Kp * error + Kd * derivative;
		motorSetVelocityAtVolts(2, velocity, powerLevelMain() / 1000.0);

		lastT = t;
		lastError = error;

		delay(10);
	}
*/
}
