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
#include "Encoder.hpp"
#include "EncoderWheel.hpp"
#include "Motor.hpp"
#include "Odometry.hpp"
#include "PidController.hpp"

#include <cmath>

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
	const bns::Motor driveMotorL(1);
	const bns::Motor driveMotorL2(2);
	const bns::Motor driveMotorR(3, true);
	const bns::Motor driveMotorR2(4, true);

	const bns::Encoder encoderL(1, 2);
	const bns::Encoder encoderR(3, 4);
	const bns::Encoder encoderM(5, 6);

	const bns::EncoderWheel encoderWheelL(encoderL, 360, 3.25, 5);
	const bns::EncoderWheel encoderWheelR(encoderR, 360, 3.25, 5);
	const bns::EncoderWheel encoderWheelM(encoderM, 360, 3.25, 5);
pros::printf("16\n");
	bns::Odometry odometry(encoderWheelL, encoderWheelR, encoderWheelM, 8);

	bns::PidController driveController(1, 0, 0);
	bns::PidController turnController(0.1, 0, 0);
pros::printf("17\n");
	const bns::Pose target(10, 0, 0);
	const unsigned long t0 = pros::micros();
	unsigned long t;

	while (true) {
pros::printf("1\n");
		t = pros::micros() - t0;
		odometry.computePose().translationToPoint(target);
		bns::Pose translation;//(odometry.computePose().translationToPoint(target));
pros::printf("2\n");
//pros::printf("x: %f, y: %f, theta: %f\n", odometry.pose().x(), odometry.pose().y(), odometry.pose().theta());
		double drive = driveController.computeOutput(std::hypot(translation.x(), translation.y()), t);
pros::printf("3\n");
		double turn = turnController.computeOutput(translation.theta(), t);
pros::printf("4\n");

		double left = drive - turn;
		double right = drive + turn;
pros::printf("5\n");

		driveMotorL.setPower(left);
pros::printf("6\n");
		driveMotorL2.setPower(left);
pros::printf("7\n");
		driveMotorR.setPower(right);
pros::printf("8\n");
		driveMotorR2.setPower(right);
pros::printf("9\n");

		pros::delay(200);
pros::printf("10\n");
	}
}
