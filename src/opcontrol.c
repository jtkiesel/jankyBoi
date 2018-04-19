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

		int val = ultrasonicGet(front_left_sonar);
		static int good_count = 0; //????
		//printf("val = %d \n", val);

		if (joystickGetDigital(1, 7, JOY_UP))
		{
			PidController straight_controller = pidControllerCreate(2, 0, 0);
			PidController naive_turn_controller = pidControllerCreate(1.5, 0, 0);
			PidController short_turn_controller = pidControllerCreate(2.4, 0, 0.05);
			PidController normal_turn_controller = pidControllerCreate(1.8, 0, 0.11);
			PidController normal_mogo_turn_controller = pidControllerCreate(1.9, 0, 0.12);

			navigator.straightController = straight_controller;
			navigator.turnController = normal_turn_controller;

			xsens_reset_heading(&xsens, 0, 0, 0);

			// FIRST MOGO
			mogoDown();
			navigatorDriveToDistance(&navigator, 15, 0, 0.8, 0.8);
			navigatorDriveToDistanceUntil(&navigator, 40, 0, 0.9, 0.9, UNTIL_LEFT_LINE);
			navigatorDriveToDistance(&navigator, 15, 0, 0.9, 0.05);
			mogoUp();
			delay(500);

			navigator.turnController = normal_mogo_turn_controller;
			navigatorDriveToDistanceUntil(&navigator, -40, 0, 0.9, 0.8, UNTIL_LEFT_LINE);
			navigatorDriveToDistance(&navigator, -25, -0.3, 0.6, 0.05);
			navigatorTurnToAngle(&navigator, toRadians(-153), 0.8, 0.05);//-150(-155)
			mogoDown();
			navigatorDriveToDistance(&navigator, 7, toRadians(-153), 0.8, 0.1);//4,-150,0.6,.0.05(6,-155,0.7,0.05)
			//delay(750);

			navigatorDriveToDistance(&navigator, -10, toRadians(-150), 0.6, 0.05);

			navigator.turnController = normal_turn_controller;
			navigatorTurnToAngle(&navigator, toRadians(70), 0.8, 0.05);
			navigatorDriveToDistance(&navigator, 20, toRadians(67), 0.8, -0.05);

			mogoUp();

			navigator.turnController = normal_mogo_turn_controller;

			navigatorTurnToAngle(&navigator, toRadians(-155), 0.8, -0.05);
			navigatorDriveToDistance(&navigator, 35, toRadians(-155), 0.8, 0.8);
			navigatorSmoothTurnToAngle(&navigator, SMOOTH_TURN_LEFT, toRadians(-135), 0.9, 0.3, 0.9);
			navigatorDriveToDistance(&navigator, 15, toRadians(-135), 1.0, 0.0);

			mogoDown();

			navigatorDriveToDistanceUntil(&navigator, -17, toRadians(-135), 1.0, 0.1, UNTIL_BACK_LINE);

			// Line up and get mogo 3
			//xsens_reset_heading(&xsens, 0, 0, -135);







			mogoUp();

			navigator.turnController = normal_turn_controller;
			navigatorTurnToAngle(&navigator, toRadians(135), 0.8, -0.05);

			navigator.until_target = 46;
			navigatorDriveToDistanceUntil(&navigator, 80, toRadians(135), 0.6, -0.05, UNTIL_FRONT_LEFT_SONAR);

			navigatorSmoothTurnToAngle(&navigator, SMOOTH_TURN_RIGHT, toRadians(90), 0.8, -0.1, 0);
			mogoDown();
			navigatorDriveToDistanceUntil(&navigator, 25, toRadians(90), 0.8, -0.05, UNTIL_RIGHT_LINE);
			navigatorDriveToDistance(&navigator, 20, toRadians(90), 0.8, 0.1);

			// Pickup Mogo 3
			mogoUp();
			delay(500);

			navigatorTurnToAngle(&navigator, toRadians(100), 0.8, -0.05);

			navigatorDriveToDistanceUntil(&navigator, -40, toRadians(100), 0.8, -0.05, UNTIL_LEFT_LINE);
			navigatorDriveToDistance(&navigator, -30, toRadians(100), 0.8, 0.05);

			navigatorTurnToAngle(&navigator, toRadians(-135), 0.8, -0.05);
			navigatorDriveToDistance(&navigator, 12, toRadians(-135), 0.8, -0.1);//7,-135,0.8,-0.05

			// Score Mogo 3
			mogoDown();
			delay(750);

			navigatorDriveToDistance(&navigator, -12, toRadians(-135), 0.8, 0.05);//-7,-135,0.5,0.05

			navigatorTurnToAngle(&navigator, toRadians(135), 0.8, 0.8);
			navigatorTurnToAngle(&navigator, toRadians(45), 0.8, -0.05);

			navigatorDriveToDistance(&navigator, 21, toRadians(45), 0.8, 0.05);

			// Picks up 4
			mogoUp();
			delay(500);

			navigatorDriveToDistance(&navigator, -25, toRadians(45), 0.8, 0.05);

			navigator.turnController = normal_mogo_turn_controller;
			navigatorTurnToAngle(&navigator, toRadians(-115), 0.8, -0.05);
			navigatorDriveToDistance(&navigator, 11, toRadians(-115), 0.8, 0.05);//10,-115,0.8,0.05

			// Scores 4
			mogoDown();
			delay(500);

			navigatorDriveToDistance(&navigator, -8, toRadians(-110), 0.8, 0.05);//-7,-110,0.8,0.05
			navigatorTurnToAngle(&navigator, toRadians(135), 0.8, -0.05);
			navigatorTurnToAngle(&navigator, toRadians(45), 0.8, -0.05);
			navigatorDriveToDistance(&navigator, 57, toRadians(45), 0.8, 0.05);

			mogoUp();

			navigatorDriveToDistance(&navigator, 18, toRadians(45), 0.8, 0.05);

			// Score #5
			navigatorTurnToAngle(&navigator, toRadians(25), 0.8, -0.05);
			navigatorDriveToDistance(&navigator, 25, toRadians(25), 0.8, 0.8);
			navigatorSmoothTurnToAngle(&navigator, SMOOTH_TURN_LEFT, toRadians(45), 0.9, 0.3, 0.9);
			navigatorDriveToDistance(&navigator, 15, toRadians(45), 1.0, 0.0);

			mogoDown();
			delay(500);

			navigatorDriveToDistanceUntil(&navigator, -17, toRadians(45), 1.0, 0.1, UNTIL_BACK_LINE);

			// DELETE LATER
			/*xsens_reset_heading(&xsens, 0, 0, 45);
			// ----

			//  #6
			mogoUp();

			navigator.turnController = normal_turn_controller;
			navigatorTurnToAngle(&navigator, toRadians(135), 0.8, -0.05);

			navigator.until_target = 46;
			navigatorDriveToDistanceUntil(&navigator, 80, toRadians(135), 0.6, -0.05, UNTIL_FRONT_RIGHT_SONAR);

			navigatorSmoothTurnToAngle(&navigator, SMOOTH_TURN_LEFT, toRadians(180), 0.8, -0.1, 0);
			mogoDown();
			navigatorDriveToDistance(&navigator, 6, toRadians(180), 0.8, -0.05);
			navigatorDriveToDistanceUntil(&navigator, 25, toRadians(180), 0.8, -0.05, UNTIL_LEFT_LINE);
			navigatorDriveToDistance(&navigator, 20, toRadians(180), 0.8, 0.1);

			// Pickup Mogo 6
			mogoUp();
			delay(500);

			navigatorTurnToAngle(&navigator, toRadians(170), 0.8, -0.05);

			navigatorDriveToDistanceUntil(&navigator, -40, toRadians(170), 0.8, -0.05, UNTIL_RIGHT_LINE);
			navigatorDriveToDistance(&navigator, -30, toRadians(170), 0.8, 0.05);

			navigatorTurnToAngle(&navigator, toRadians(45), 0.8, -0.05);
			navigatorDriveToDistance(&navigator, 8, toRadians(45), 0.8, -0.05);

			// Score Mogo 6
			mogoDown();
			delay(750);

			navigatorDriveToDistance(&navigator, -7, toRadians(45), 0.8, 0.05);


			// #7
			mogoUp();

			navigator.turnController = normal_turn_controller;
			navigatorTurnToAngle(&navigator, toRadians(-45), 0.8, -0.05);

			navigator.until_target = 46;
			navigatorDriveToDistanceUntil(&navigator, 80, toRadians(-45), 0.6, -0.05, UNTIL_FRONT_LEFT_SONAR);

			navigatorSmoothTurnToAngle(&navigator, SMOOTH_TURN_RIGHT, toRadians(-90), 0.8, -0.1, 0);
			mogoDown();
			navigatorDriveToDistanceUntil(&navigator, 25, toRadians(-90), 0.8, -0.05, UNTIL_RIGHT_LINE);
			navigatorDriveToDistance(&navigator, 20, toRadians(-90), 0.8, 0.1);

			// Pickup Mogo 7
			mogoUp();
			delay(1500);

			navigatorTurnToAngle(&navigator, toRadians(-80), 0.8, -0.05);

			navigatorDriveToDistanceUntil(&navigator, -40, toRadians(-80), 0.8, -0.05, UNTIL_LEFT_LINE);
			navigatorDriveToDistance(&navigator, -30, toRadians(-80), 0.8, 0.05);

			navigatorTurnToAngle(&navigator, toRadians(45), 0.8, -0.05);
			navigatorDriveToDistance(&navigator, 8, toRadians(45), 0.8, -0.05);//7,45,0.8,-0.05

			// Score Mogo 7
			mogoDown();
			delay(750);

			navigatorDriveToDistance(&navigator, -8, toRadians(45), 0.8, 0.05);//-7,45,0.8,0.05

			// Mogo 8
			navigatorTurnToAngle(&navigator, toRadians(-45), 0.8, 0.8);
			navigatorTurnToAngle(&navigator, toRadians(-125), 0.8, -0.05);

			navigatorDriveToDistance(&navigator, 21, toRadians(-125), 0.8, 0.05);

			// Picks up 8
			mogoUp();
			delay(500);

			navigatorDriveToDistance(&navigator, -25, toRadians(-135), 0.8, 0.05);

			navigator.turnController = normal_mogo_turn_controller;
			navigatorTurnToAngle(&navigator, toRadians(70), 0.8, -0.05);
			navigatorDriveToDistance(&navigator, 10, toRadians(70), 0.8, 0.05);

			// Scores 8
			mogoDown();
			delay(500);

			navigatorDriveToDistance(&navigator, -7, toRadians(70), 0.8, 0.05);*/

		}
		if (joystickGetDigital(1, 8, JOY_UP)) {
			mogoUp();
		} else if (joystickGetDigital(1, 8, JOY_RIGHT)) {
			mogoDown();
		}

		int driveL = joystickGetAnalog(1, 3);
		int driveR = joystickGetAnalog(1, 2);
		driveSetPwm(&drive, driveL, driveR);
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
