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

#include "api.hpp"
#include "constants.hpp"
#include "main.hpp"
#include "PathFollowerTest.hpp"

#include <cmath>

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
	if (power == 0.0) {
		return 0;
	}
	double p = std::abs(power);
	if (p >= 1.0) {
		return (int)std::round(std::copysign(127.0, power));
	}
	return (int)std::round(std::copysign(((((((((-44128.10541 * p + 178572.6802) * p
			- 297071.4563) * p + 262520.7547) * p - 132692.6561) * p + 38464.48054) * p
			- 6049.717501) * p + 476.2279947) * p - 1.233957961), power));
}

void motorSetLinear(unsigned char port, double power) {
	pros::motorSet(port, powerToPwm(power));
}

void motorSetAtVolts(unsigned char port, double power, double volts) {
	motorSetLinear(port, batteryAdjustedPower(power, volts));
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
	bns::testPathFollower();
	while (true) {
		pros::delay(20);
	}
}
