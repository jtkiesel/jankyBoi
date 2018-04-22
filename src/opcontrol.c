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
#include "EncoderAnalog.h"
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

	//taskRunLoop(compControlTask, 100);
	taskRunLoop(odometryTask, 2);
	taskRunLoop(debugTask, 100);
	taskRunLoop(mogoTask, 20);

	taskRunLoop(liftTask, 20);
	taskRunLoop(intakeTask, 50);
	taskRunLoop(encoderAnalogTask, 5);

  	//liftDown();
	//intakeIn();
	//Pose point = {.x = 20.0, .y = 20.0, .theta = 0.0};
	//navigatorTurnToPoint(&navigator, point, 1.0, 0.0);
	//navigatorDriveToPoint(&navigator, point, 1.0, 0.0);
	//delay(1000);
	//point = (Pose) {.x = 0.0, .y = 0.0, .theta = 0.0};
	//navigatorDriveToPoint(&navigator, point, -1.0, 0.0);

	taskCreate(pidTuneTask, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);

	double targetAngle = 0.0;

	while (true) {
		//printf("encoderRoller = %d\n", encoder1WireCounts(encoderRoller));
		//printf("ultrasonicLeft = %d\n", ultrasonicGet(front_left_sonar));
	//	printf("ultrasonicRight = %d\n", ultrasonicGet(front_right_sonar));
		//int val = ultrasonicGet(front_left_sonar);
		//printf("val = %d \n", val);



		  int val = analogRead(leftBarDetect.port);
		  printf("Bar = %d %d\n", val, analogRead(5));

		if (joystickGetDigital(1, 7, JOY_DOWN)) {
//			targetAngle += toRadians(90);
//			navigatorTurnToAngle(&navigator, targetAngle, 1.0, 0.0);
			xsens_reset_heading(&xsens, 0, 0, 0);


			liftMid();

			navigator.until_target = 47;
			navigatorDriveToDistanceUntil(&navigator, 80, toRadians(0), 0.6, -0.2, UNTIL_FRONT_RIGHT_SONAR);
			delay(100);
			navigatorTurnToAngle(&navigator, toRadians(45), 0.6, -0.1);

			mogoDown();

			navigatorDriveToDistanceUntil(&navigator, 80, toRadians(45), 0.6, -0.2, UNTIL_LEFT_LINE);
			navigatorDriveToDistanceUntil(&navigator, 40, toRadians(45), 0.9, -0.2, UNTIL_MOGO_FOUND);
			mogoUp();
			navigatorDriveToDistance(&navigator, 5, toRadians(45), 0.9, 0.1);

			delay(200);

			//drivers backwards after picking up mogo

			navigatorTurnToAngle(&navigator, toRadians(25), 0.9, -0.1);
			navigatorDriveToDistanceUntil(&navigator, -30, toRadians(25), 0.9, -0.2, UNTIL_RIGHT_LINE);
			navigatorDriveToDistance(&navigator, -10, toRadians(25), 0.9, 0.2);
			navigatorTurnToAngle(&navigator, toRadians(-105), 0.9, -0.1);

			mogoDown();
			navigatorDriveToDistanceUntil(&navigator, 80, toRadians(-105), 0.9, -0.2, UNTIL_LEFT_LINE);
			navigatorDriveToDistanceUntil(&navigator, 80, toRadians(-105), 0.9, -0.2, UNTIL_LEFT_BAR);
			delay(500);

			//score goal

			navigatorDriveToDistance(&navigator, -5, toRadians(-105), 0.9, 0.2);
			delay(100);
			mogoUp();

			//turn towards mogo 7

			printf("---Mogo7---");
			navigatorTurnToAngle(&navigator, toRadians(180), 0.9, -0.1);
			navigatorDriveToDistance(&navigator, 10, toRadians(180), 0.9, 0);
			navigator.until_target = 40;

			liftUp();
			navigatorDriveToDistanceUntil(&navigator, 80, toRadians(180), 0.6, -0.2, UNTIL_FRONT_LEFT_SONAR);
			delay(100);

			intakeIn();

			mogoDown();
			navigatorTurnToAngle(&navigator, toRadians(140), 0.6, -0.1);



			navigatorDriveToDistanceUntil(&navigator, 80, toRadians(140), 0.9, -0.2, UNTIL_RIGHT_LINE);
			navigatorDriveToDistanceUntil(&navigator, 40, toRadians(137), 0.9, -0.2, UNTIL_MOGO_FOUND);
			mogoUp();
			liftDown();

			navigatorDriveToDistance(&navigator, 5, toRadians(140), 0.9, 0.1);
			delay(200);

			//drivers backwards after picking up mogo

			navigatorTurnToAngle(&navigator, toRadians(155), 0.9, -0.1);
			navigatorDriveToDistance(&navigator, -30, toRadians(155), 0.9, -0.2);
			navigatorTurnToAngle(&navigator, toRadians(-75), 0.9, -0.1);

			intakeOut();
			liftMid();

			mogoDown();

			navigatorDriveToDistanceUntil(&navigator, 80, toRadians(-75), 0.9, 0.2, UNTIL_RIGHT_BAR);

			delay(100);

			navigatorDriveToDistance(&navigator, -2, toRadians(-75), 0.9, 0.2);
			delay(100);

		}
		odometryUseXsens(navigator.odometry);

		if (joystickGetDigital(1, 7, JOY_UP)) {
			PidController straight_controller = pidControllerCreate(4, 0, 0);
			PidController naive_turn_controller = pidControllerCreate(1.5, 0, 0);
			PidController short_turn_controller = pidControllerCreate(2.4, 0, 0.05);
			PidController normal_turn_controller = pidControllerCreate(1.8, 0, 0.11);
			PidController normal_mogo_turn_controller = pidControllerCreate(1.9, 0, 0.12);

			navigator.straightController = straight_controller;
			navigator.turnController = normal_turn_controller;

			/*xsens_reset_heading(&xsens, 0, 0, 0);

			// FIRST MOGO
			mogoDown();
			liftMid();
			navigatorDriveToDistance(&navigator, 15, 0, 0.95, 0.95);
			navigatorDriveToDistanceUntil(&navigator, 40, 0, 0.95, 0.95, UNTIL_RIGHT_LINE);
			navigatorDriveToDistance(&navigator, 17, 0, 0.95, 0.05);
			mogoUp();
			delay(500);

			navigator.turnController = normal_mogo_turn_controller;
			navigatorDriveToDistanceUntil(&navigator, -80, 0, 0.9, 0.8, UNTIL_RIGHT_LINE);
			navigatorSmoothTurnToAngle(&navigator, SMOOTH_TURN_RIGHT, toRadians(-0.7), -0.9, 0.3, 0.9);
			liftDown();
			navigatorDriveToDistance(&navigator, -20, -0.7, 0.6, 0.1);

			intakeOut();
			liftUp();
			navigatorTurnToAngle(&navigator, toRadians(-157), 0.8, 0.05);//-150(-155)

			// Score #1
			mogoDown();
			navigatorDriveToDistanceUntil(&navigator, 30, toRadians(-157), 0.8, -0.1, UNTIL_LEFT_BAR);//4,-150,0.6,.0.05(6,-155,0.7,0.05)
			delay(350);

			navigatorDriveToDistance(&navigator, -7, toRadians(-157), 0.6, 0.05);

			// Go to Mogo #2
			navigator.turnController = normal_turn_controller;
			navigatorTurnToAngle(&navigator, toRadians(73), 0.8, 0.05);
			intakeIn();
			navigatorDriveToDistance(&navigator, 15, toRadians(73), 0.8, 0);

			// Pick up mogo 2
			mogoUp();

			navigator.turnController = normal_mogo_turn_controller;

			navigatorDriveToDistance(&navigator, 13, toRadians(73), 0.5, -0.05);

			delay(500);
			liftDown();
			navigatorDriveToDistance(&navigator, -25, toRadians(73), 0.8, 0.05);

			navigatorTurnToAngle(&navigator, toRadians(180), 0.8, 0.05);

			navigatorDriveToDistance(&navigator, 11, toRadians(180), 0.9, -0.05);
			intakeOut();
			liftMid();

			navigatorSmoothTurnToAngle(&navigator, SMOOTH_TURN_LEFT, toRadians(-135), 0.9, 0.3, 0.9);
			mogoDown();
			navigatorDriveToDistance(&navigator, 5, toRadians(-135), 1.0, -0.05);
			navigatorDriveToDistanceUntil(&navigator, 15, toRadians(-135), 1.0, 1.0, UNTIL_RIGHT_BAR);

			// Score #2
			delay(250);

			navigatorDriveToDistance(&navigator, -15, toRadians(-135), 0.9, -0.05);
*/
			// Line up and get mogo 3
		/*	xsens_reset_heading(&xsens, 0, 0, -135);
			odometryUseXsens(navigator.odometry);
			mogoDown();
			liftMid();

			mogoUp();

			liftUp();

			navigator.turnController = normal_turn_controller;
			navigatorTurnToAngle(&navigator, toRadians(135), 0.8, -0.05);

			delay(250);

			navigator.until_target = 56;
			navigatorDriveToDistanceUntil(&navigator, 80, toRadians(135), 0.4, -0.05, UNTIL_FRONT_LEFT_SONAR);

			//navigatorTurnToAngle(&navigator, toRadians(90), 0.8, -0.05);


			navigatorSmoothTurnToAngle(&navigator, SMOOTH_TURN_RIGHT, toRadians(90), 0.4, -0.1, 0);
			mogoDown();

			intakeIn();
			navigatorDriveToDistance(&navigator, 7, toRadians(90), 0.8, -0.05);
			navigatorDriveToDistanceUntil(&navigator, 25, toRadians(90), 0.4, -0.05, UNTIL_RIGHT_LINE);
			navigatorDriveToDistance(&navigator, 15, toRadians(90), 0.4, 0.1);

			// Pickup Mogo 3
			liftDown();
			mogoUp();
			delay(500);

			navigatorTurnToAngle(&navigator, toRadians(100), 0.8, -0.05);

			navigatorDriveToDistanceUntil(&navigator, -40, toRadians(105), 0.8, -0.05, UNTIL_LEFT_LINE);
			navigatorDriveToDistance(&navigator, -26, toRadians(105), 0.8, 0.05);
			intakeOut();
			liftMid();

			navigatorTurnToAngle(&navigator, toRadians(-135), 0.8, -0.05);
			mogoDown();
			navigatorDriveToDistanceUntil(&navigator, 15, toRadians(-135), 0.8, -0.1, UNTIL_LEFT_BAR);//7,-135,0.8,-0.05

			// Score Mogo 3
			delay(500);

			navigatorDriveToDistance(&navigator, -8, toRadians(-135), 0.4, 0.1);//-7,-135,0.5,0.05

			intakeIn();

			navigatorTurnToAngle(&navigator, toRadians(135), 0.8, 0.8);
			navigatorTurnToAngle(&navigator, toRadians(45), 0.8, -0.05);

			liftUp();
			navigatorDriveToDistance(&navigator, 21, toRadians(45), 0.8, 0.05);

			// Picks up 4
			mogoUp();
			navigatorDriveToDistance(&navigator, 10, toRadians(45), 0.8, -0.05);
			delay(750);

			liftDown();
			delay(750);
			navigatorDriveToDistance(&navigator, -25, toRadians(45), 0.8, 0.05);

			navigator.turnController = normal_mogo_turn_controller;
			navigatorTurnToAngle(&navigator, toRadians(-115), 0.8, -0.05);
			intakeOut();
			liftMid();
			navigatorDriveToDistanceUntil(&navigator, 11, toRadians(-115), 0.8, 0.05, UNTIL_RIGHT_BAR);//10,-115,0.8,0.05

			// Scores 4
			mogoDown();
			delay(750);

			navigatorDriveToDistance(&navigator, -8, toRadians(-110), 0.8, 0.05);//-7,-110,0.8,0.05

			navigator.turnController = normal_turn_controller;
			navigatorTurnToAngle(&navigator, toRadians(135), 0.8, -0.05);
			navigatorDriveToDistance(&navigator, -3, toRadians(135), 0.8, 0.05);//-7,-110,0.8,0.05
			navigatorTurnToAngle(&navigator, toRadians(45), 0.8, -0.05);
			liftUp();
			intakeIn();
			navigatorDriveToDistance(&navigator, 55, toRadians(45), 0.8, -0.05);

			// Pick up 5
			mogoUp();
			liftDown();

			delay(1000);

			navigatorDriveToDistance(&navigator, 13, toRadians(45), 0.8, 0.05);

			// Score #5
			navigatorTurnToAngle(&navigator, toRadians(0), 0.8, -0.05);
			intakeOut();
			liftMid();
			navigatorSmoothTurnToAngle(&navigator, SMOOTH_TURN_LEFT, toRadians(45), 1.0, 0.1, 1.0);
			mogoDown();
			navigatorDriveToDistanceUntil(&navigator, 15, toRadians(45), 1.0, 1.0, UNTIL_RIGHT_BAR);

			delay(250);

			navigatorDriveToDistance(&navigator, -12, toRadians(45), 1.0, 0.1);

			*/

			// DELETE LATER
			xsens_reset_heading(&xsens, 0, 0, 45);
			odometryUseXsens(navigator.odometry);
			mogoUp();
			liftMid();
			// ----

			//  #6
			mogoUp();
			liftLoads();

			navigator.turnController = normal_turn_controller;
			navigatorTurnToAngle(&navigator, toRadians(135), 0.8, -0.05);

			navigator.until_target = 55;
			navigatorDriveToDistanceUntil(&navigator, 80, toRadians(135), 0.6, -0.05, UNTIL_FRONT_RIGHT_SONAR);

			navigatorSmoothTurnToAngle(&navigator, SMOOTH_TURN_LEFT, toRadians(180), 0.8, -0.1, 0);
			mogoDown();
			navigatorDriveToDistance(&navigator, 6, toRadians(180), 0.8, -0.05);
			navigatorDriveToDistanceUntil(&navigator, 25, toRadians(180), 0.8, -0.05, UNTIL_LEFT_LINE);
			navigatorDriveToDistance(&navigator, 15, toRadians(180), 0.8, 0.1);

			// Pickup Mogo 6
			mogoUp();
			delay(500);

			navigatorTurnToAngle(&navigator, toRadians(160), 0.8, -0.05);

			navigatorDriveToDistanceUntil(&navigator, -40, toRadians(160), 0.6, 0.2, UNTIL_RIGHT_LINE);
			delay(100);
			navigatorDriveToDistanceUntil(&navigator, 10, toRadians(160), 0.3, 0.1, UNTIL_RIGHT_LINE);
			delay(200);
			liftPickupLoads();
			intakeIn();
			delay(200);
			liftDown();
			/*navigatorDriveToDistance(&navigator, -30, toRadians(170), 0.8, 0.05);

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
		if (joystickGetDigital(1, 6, JOY_UP)) {
			liftMid();
		} else if (joystickGetDigital(1, 6, JOY_DOWN)) {
			liftDown();
		} else if (joystickGetDigital(1, 5, JOY_UP)) {
			liftUp();
		}

		int driveL = joystickGetAnalog(1, 3);
		int driveR = joystickGetAnalog(1, 2);
		driveSetPwm(&drive, driveL, driveR);
		driveL = joystickGetAnalog(1, 3);
		driveR = joystickGetAnalog(1, 2);
		//lift = joystickGetDigital(1, 5, JOY_UP) ? 127 : (joystickGetDigital(1, 5, JOY_DOWN) ? -127 : 0);
		//rollers = joystickGetDigital(1, 6, JOY_UP) ? 127 : (joystickGetDigital(1, 6, JOY_DOWN) ? -127 : 15);
		//mogo = joystickGetDigital(1, 8, JOY_UP) ? 127 : (joystickGetDigital(1, 8, JOY_RIGHT) ? -127 : -15);

		driveSetPwm(&drive, driveL, driveR);
		//motorSetPwm(&motorLift, lift);
		//motorSetPwm(&motorRollers, rollers);
		//motorSetPwm(&motorMogo, mogo);

		delay(20);
	}
}
