/** @file opcontrol.c
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

#include "main.h"

#include "API.h"
#include "globals.h"
#include "Motor.h"
#include "Navigator.h"
#include "Odometry.h"
#include "Pose.h"
#include "xsens.h"
#include "util.h"

#include <math.h>

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
	int driveL, driveR;
	int lift;
	int rollers;
	int mogo;

	taskRunLoop(compControlTask, 100);
	taskRunLoop(odometryTask, 2);
	taskRunLoop(debugTask, 100);
	taskRunLoop(mogoTask, 20);

	//taskRunLoop(liftTask, 20);
	taskRunLoop(intakeTask, 50);



  	//liftDown();
	//intakeIn();
	//Pose point = {.x = 20.0, .y = 20.0, .theta = 0.0};
	//navigatorTurnToPoint(&navigator, point, 1.0, 0.0);
	//navigatorDriveToPoint(&navigator, point, 1.0, 0.0);
	//delay(1000);
	//point = (Pose) {.x = 0.0, .y = 0.0, .theta = 0.0};
	//navigatorDriveToPoint(&navigator, point, -1.0, 0.0);

	while (true) {
		if (joystickGetDigital(1, 7, JOY_UP))
		{
			PidController straight_controller = pidControllerCreate(4, 0, 0);
			PidController naive_turn_controller = pidControllerCreate(1.5, 0, 0);
			PidController short_turn_controller = pidControllerCreate(2.2, 0, 0.07);
			PidController normal_turn_controller = pidControllerCreate(1.7, 0, 0.12);
			PidController normal_mogo_turn_controller = pidControllerCreate(1.7, 0, 0.15);

			// FIRST MOGO
			/*mogoDown();
			navigatorDriveToDistance(&navigator, 5, 0, 0.8, 0.8);
			navigatorDriveToDistanceUntil(&navigator, 40, 0, 0.8, 0.8, UNTIL_LEFT_LINE);
			navigatorDriveToDistance(&navigator, 25, 0, 0.8, -0.05);
			mogoUp();
			delay(1500);
			navigatorDriveToDistanceUntil(&navigator, -40, 0, 0.8, 0.8, UNTIL_LEFT_LINE);
			navigatorDriveToDistance(&navigator, -20, -0.8, 0.6, 0.05);
			navigatorTurnToAngle(&navigator, -2.44, 0.8, 0.05);
			mogoDown();
			navigatorDriveToDistance(&navigator, 5, -2.44, 0.6, 0.05);
			delay(500);

			navigatorDriveToDistance(&navigator, -40, 3.1415, 0.6, 0.05);
			*/

			// Delete Later
			//mogoDown();
			xsens_reset_heading(&xsens, 0, 0, 180);

			// ----
			//navigatorTurnToAngle(&navigator, 2.35, 0.8, -0.05);
			//navigator.straightController = straight_controller;
			//navigatorDriveToDistance(&navigator, 80, 3.1415, 0.6, -0.05);

			mogoDown();
			navigator.straightController = straight_controller;

			navigator.turnController = short_turn_controller;
			navigatorTurnToAngle(&navigator, 2.68, 0.8, -0.1);
			navigatorDriveToDistance(&navigator, 24, 2.68, 0.9, -0.05);

			mogoUp();
			delay(500);
			navigatorDriveToDistance(&navigator, -4, 2.68, 0.9, 0.05);

			navigator.turnController = normal_mogo_turn_controller;
			navigatorTurnToAngle(&navigator, toRadians(-155), 0.8, -0.05);
			navigatorDriveToDistance(&navigator, 45, toRadians(-155), 0.9, 0.9);
			navigatorDriveToDistance(&navigator, 18, toRadians(-135), 0.9, -0.05);
			mogoDown();
			delay(500);
			navigatorDriveToDistance(&navigator, -12, toRadians(-135), 1.0, 0.05);
			mogoUp();
			navigatorTurnToPoint(&navigator, (Pose){.x = -12, .y = 80}, 0.8, 0.05);
			navigatorDriveToPoint(&navigator, (Pose){.x = -12, .y = 80}, 0.8, 0.05);

			/*navigatorTurnToAngle(&navigator, 2.6, 1.0, 0.05);
			navigatorDriveToDistance(&navigator, 15, 2.6, 0.6, -0.05);
			mogoUp();
			delay(1000);
			navigatorTurnToAngle(&navigator, -2.8, 1.0, 0.05);
			navigatorDriveToDistance(&navigator, 15, -2.8, 0.8, -0.05);
			navigatorDriveToDistance(&navigator, 15, 3.4, 0.6, -0.05);
			navigatorDriveToDistance(&navigator, 15, -2.8, 0.6, -0.05);
			mogoDown();*/

			//navigatorTurnToAngle(&navigator, -1.2, -0.8, -0.05);
		}
		if (joystickGetDigital(1, 8, JOY_UP)) {
			mogoUp();
		} else if (joystickGetDigital(1, 8, JOY_RIGHT)) {
			mogoDown();
		}
		/*if (joystickGetDigital(1, 5, JOY_UP)) {
			//navigatorDriveToPoint(&navigator, (Pose) {.x = 30.0, .y = 0.0, .theta = 0.0}, 1.0, 0.0);
		} else {
			driveL = joystickGetAnalog(1, 3);
			driveR = joystickGetAnalog(1, 2);
			lift = joystickGetDigital(1, 5, JOY_UP) ? 127 : (joystickGetDigital(1, 5, JOY_DOWN) ? -127 : 0);
			rollers = joystickGetDigital(1, 6, JOY_UP) ? 127 : (joystickGetDigital(1, 6, JOY_DOWN) ? -127 : 15);
			mogo = joystickGetDigital(1, 8, JOY_UP) ? 127 : (joystickGetDigital(1, 8, JOY_RIGHT) ? -127 : -15);

			driveSetPwm(&drive, driveL, driveR);
			motorSetPwm(&motorLift, lift);
			motorSetPwm(&motorRollers, rollers);
			motorSetPwm(&motorMogo, mogo);
		}*/

		delay(20);
	}
}
