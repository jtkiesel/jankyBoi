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

PidController straight_controller;
PidController naive_turn_controller;
PidController short_turn_controller;
PidController normal_turn_controller;
PidController normal_right_turn_controller;
PidController normal_mogo_turn_controller;
PidController double_mogo_turn_controller;

void PSC_score_right_wall(double offset)
{
	// Score Mogo
	printf("Until left bar");

	navigatorDriveToDistance(&navigator, 7, toRadians(-145+offset), 0.4, -0.1);
	mogoDownSlow();
	navigatorDriveToDistanceUntil(&navigator, 30, toRadians(-145+offset), 0.4, -0.1, UNTIL_LEFT_BAR);//4,-150,0.6,.0.05(6,-155,0.7,0.05)
	navigatorDriveForTime(&navigator, 0.3, 0.3, 700);

	navigator.turnController = normal_mogo_turn_controller;

	waitUntilMogo();
//	mogoUp();
	navigatorDriveToDistance(&navigator, -5, toRadians(-145+offset), 0.6, 0.2);
	mogoUp();
	navigatorDriveToDistance(&navigator, -5, toRadians(-145+offset), 0.6, 0.2);
	delay(100);
	navigator.turnController = normal_turn_controller;

	navigatorTurnToAngle(&navigator, toRadians(-45+offset), 0.7, -0.1);//-150(-155)
	delay(100);
	navigator.until_target = -43;
	navigatorDriveToDistanceUntil(&navigator, -40, toRadians(-45+offset), 0.6, 0.2, UNTIL_FRONT_RIGHT_SONAR);
	//navigatorDriveToDistanceUntil(&navigator, 30, toRadians(-45), 0.6, -0.1, UNTIL_FRONT_LEFT_SONAR);
	delay(100);
	navigatorTurnToAngle(&navigator, toRadians(45+offset), 0.7, -0.1);

	delay(100);
	driveSetPower(navigator.drive, 0, 0);
}

void PSC_loader()
{
	intakeIn();
	liftPickupLoads();

	delay(400);

	for (int i = 0; i < 3; i++)
	{
		intakeIn();
		waitUntilLift();
		liftDown();
		waitUntilLift();
		intakeOut();
		liftPickupLoads();
		delay(200);
	}

	liftMid();
}

void PSC_first_mogo()
{
	mogoDown();
	intakeIn();
	liftMid();
	//delay(250);
	navigatorDriveToDistance(&navigator, 15, 0, 1, 1);
	//navigatorDriveToDistanceUntil(&navigator, 40, 0, 0.95, 0.95, UNTIL_RIGHT_LINE);
	navigatorDriveToDistanceUntil(&navigator, 80, 0, 1, -0.1, UNTIL_MOGO_FOUND);
	//navigatorDriveToDistance(&navigator, 17, 0, 0.95, 0.05);
	mogoUp();
	delay(250);

	navigatorTurnToAngle(&navigator, toRadians(-18), 0.6, -0.1);

	navigatorDriveToDistance(&navigator, -3, toRadians(-18), 0.9, -0.9);
	liftDown();
	navigatorDriveToDistance(&navigator, -3, toRadians(-18), 0.9, -0.9);
	intakeOut();
	//navigatorDriveToDistance(&navigator, -5, toRadians(-18), 0.9, -0.9);
	liftLoads();
	navigatorDriveToDistanceUntil(&navigator, -20, toRadians(-18), 0.5, 0.1, UNTIL_RIGHT_LINE);
	//delay(200);
	navigatorDriveToDistance(&navigator, 1, toRadians(-18), 0.5, -0.1);
	//liftLoads();

	PSC_loader();

	navigatorDriveToDistance(&navigator, -30, toRadians(-18), 0.7, 0.1);
	navigatorTurnToAngle(&navigator, toRadians(-135), 0.6, 0.1);
	delay(100);

	PSC_score_right_wall(0);

	/*navigator.turnController = normal_mogo_turn_controller;
	navigatorDriveToDistanceUntil(&navigator, -80, toRadians(-4), 0.9, 0.2, UNTIL_RIGHT_LINE);
	navigatorDriveToDistance(&navigator, -13, toRadians(-4), 1, 0.05);
	delay(100);
	liftDown();
	navigatorTurnToAngle(&navigator, toRadians(-45), 0.8, 0.05);
	intakeOut();

	navigator.until_target = -45;
	navigatorDriveToDistanceUntil(&navigator, -40, toRadians(-45), 0.6, 0.2, UNTIL_FRONT_RIGHT_SONAR);

	delay(200);
	liftMid();
	navigatorTurnToAngle(&navigator, toRadians(-145), 0.8, 0.1);//-150(-155)
	mogoDown();
	delay(200);

	PSC_score_right_wall(0);*/
}

void PSC_double_mogo(double offset)
{
	digitalWrite(mogo_tipper_port, LOW);
	digitalWrite(mogo_release_tipper_port, LOW);

	navigator.turnController = normal_turn_controller;
	liftUp();
	intakeIn();
	mogoDown();
	delay(650);
	navigatorDriveToDistance(&navigator, 10, toRadians(0+offset), 0.9, -0.1);
	navigatorDriveToDistanceUntil(&navigator, 80, toRadians(0+offset), 0.9, -0.1, UNTIL_LEFT_LINE);
	navigatorDriveToDistance(&navigator, 9, toRadians(0+offset), 0.9, -0.1);
	mogoUp();

	delay(500);
	//liftMid();
	//navigatorTurnToAngle(&navigator, toRadians(10+offset), 0.8, -0.1);
//	delay(100);
	//navigatorDriveToDistance(&navigator, 5, toRadians(7+offset), 0.9, -0.1);
	intakeNone();
	liftDown();
	delay(1600);
	intakeOut();
	delay(200);
	liftUp();
	delay(800);
	intakeIn();
	delay(200);
	navigator.turnController = normal_mogo_turn_controller;

	digitalWrite(mogo_tipper_port, HIGH);
	navigatorDriveToDistance(&navigator, 4, toRadians(18+offset), 0.9, -0.1);
	navigatorTurnToAngle(&navigator, toRadians(18+offset), 1.0, -0.1);
	delay(100);
	navigatorDriveToDistance(&navigator, 15, toRadians(18+offset), 0.8, -0.25);
   // delay(250);
	digitalWrite(mogo_release_tipper_port, HIGH);
	delay(250);
	digitalWrite(mogo_tipper_port, LOW);
	navigator.turnController = double_mogo_turn_controller;
	navigatorDriveToDistance(&navigator, -3, toRadians(-20+offset), 0.9, -0.1);
	liftDown();
	navigatorTurnToAngle(&navigator, toRadians(-20+offset), 1.0, 0.1);
	navigatorDriveToDistance(&navigator, 11, toRadians(-20+offset), 0.9, -0.1);
	navigatorTurnToAngle(&navigator, toRadians(17+offset), 1.0, -0.1);
	delay(100);
	navigatorDriveToDistance(&navigator, 15, toRadians(17+offset), 0.9, -0.1);

	intakeOut();
	liftMid();

	navigatorDriveForTime(&navigator, 1.0, 1.0, 250);
	navigatorDriveForTime(&navigator, 1.0, 1.0, 1000);

	digitalWrite(mogo_tipper_port, HIGH);
	digitalWrite(mogo_release_tipper_port, LOW);

	navigatorDriveToDistance(&navigator, -5, toRadians(0+offset), 0.9, -0.1);
	mogoDown();
	navigatorDriveToDistance(&navigator, -10, toRadians(0+offset), 0.9, 0.2);
	waitUntilMogo();
	navigatorDriveToDistanceUntil(&navigator, -30, toRadians(0+offset), 0.6, -0.1, UNTIL_BACK_LINE);
	navigatorDriveToDistance(&navigator, -3, toRadians(0+offset), 0.9, -0.1);
	mogoUp();
}

void PSC_mogo_on_left_wall_single_cone(double offset)
{
	navigator.turnController = normal_right_turn_controller;

	navigatorTurnToAngle(&navigator, toRadians(135+offset), 0.7, 0.1);
	delay(200);
	liftUp();
	navigatorDriveToDistance(&navigator, 10, toRadians(135+offset), 0.7, 0);

	navigator.until_target = 38;
	navigatorDriveToDistanceUntil(&navigator, 80, toRadians(135+offset), 0.5, -0.2, UNTIL_FRONT_LEFT_SONAR);
	delay(100);

	intakeIn();

	mogoDown();
	navigatorTurnToAngle(&navigator, toRadians(95+offset), 0.6, 0.1);

	navigatorDriveToDistanceUntil(&navigator, 80, toRadians(95+offset), 0.7, -0.2, UNTIL_RIGHT_LINE);
	navigatorDriveToDistanceUntil(&navigator, 40, toRadians(95+offset), 0.7, -0.2, UNTIL_MOGO_FOUND);

	liftDown();
	mogoUp();
	delay(250);

	//drives backwards after picking up mogo

	navigatorDriveToDistanceUntil(&navigator, -40, toRadians(95+offset), 0.9, 0.2, UNTIL_LEFT_LINE);
	navigatorTurnToAngle(&navigator, toRadians(110+offset), 0.7, -0.1);
	navigatorDriveToDistance(&navigator, -9, toRadians(110+offset), 0.9, -0.2);
	intakeOut();
	liftMid();
	navigatorDriveToDistance(&navigator, -9, toRadians(110+offset), 0.9, -0.2);
	navigatorTurnToAngle(&navigator, toRadians(-125+offset), 0.7, -0.1);

	mogoDown();

	//printf("Until Right Line");
	//navigatorDriveToDistanceUntil(&navigator, 80, toRadians(-135+offset), 0.4, -0.2, UNTIL_RIGHT_LINE);
	navigatorDriveToDistance(&navigator, 4, toRadians(-125+offset), 0.6, -0.2);
	printf("Until Right Bar");
	navigatorDriveToDistanceUntil(&navigator, 80, toRadians(-125+offset), 0.6, -0.1, UNTIL_RIGHT_BAR);

	waitUntilMogo();
	//delay(100);

	navigatorDriveToDistanceUntil(&navigator, -30, toRadians(-125+offset), 0.6, -0.1, UNTIL_BACK_LINE);
}

void PSC_mogo_on_left_wall(double offset, int version)
{
	navigator.turnController = normal_right_turn_controller;

	navigatorTurnToAngle(&navigator, toRadians(135+offset), 0.7, 0.1);
	delay(200);
	navigatorDriveToDistance(&navigator, 10, toRadians(135+offset), 0.7, 0);
	navigator.until_target = 36;

	liftUp();
	navigatorDriveToDistanceUntil(&navigator, 80, toRadians(135+offset), 0.5, -0.2, UNTIL_FRONT_LEFT_SONAR);
	delay(100);

	intakeIn();

	mogoDown();
	navigatorTurnToAngle(&navigator, toRadians(95+offset), 0.6, 0.1);

	navigatorDriveToDistanceUntil(&navigator, 80, toRadians(95+offset), 0.6, -0.2, UNTIL_RIGHT_LINE);
	intakeNone();
	liftMid();
	delay(500);

	navigatorDriveToDistanceUntil(&navigator, 40, toRadians(95+offset), 0.9, -0.2, UNTIL_MOGO_FOUND);
	mogoUp();

	waitUntilMogo();
	liftDown();
	delay(300);
	intakeOut();
	delay(200);
	liftUp();
	navigatorDriveToDistance(&navigator, -10, toRadians(95+offset), 0.9, 0.1);
	intakeIn();
	delay(750);
	navigatorDriveToDistance(&navigator, 7, toRadians(95+offset), 0.9, 0.1);
	delay(500);
	intakeNone();
	liftDown();
	navigatorDriveToDistance(&navigator, -15, toRadians(95+offset), 0.9, 0.1);
	//drivers backwards after picking up mogo
	navigatorTurnToAngle(&navigator, toRadians(110+offset), 0.9, -0.1);
	navigatorDriveToDistance(&navigator, -23, toRadians(110+offset), 0.9, -0.2);
	navigatorTurnToAngle(&navigator, toRadians(-135+offset), 0.9, -0.1);

	intakeOut();
	delay(200);
	liftMid();

	mogoDown();

	printf("Until Right Line");
	//navigatorDriveToDistanceUntil(&navigator, 80, toRadians(-135+offset), 0.4, -0.2, UNTIL_RIGHT_LINE);
	navigatorDriveToDistance(&navigator, 4, toRadians(-135+offset), 0.4, -0.2);
	printf("Until Right Bar");
	navigatorDriveToDistanceUntil(&navigator, 80, toRadians(-135+offset), 0.4, -0.1, UNTIL_RIGHT_BAR);

	waitUntilMogo();
	delay(100);

	navigatorDriveToDistanceUntil(&navigator, -30, toRadians(-135+offset), 0.6, -0.1, UNTIL_BACK_LINE);
}

void PSC_mogo_on_right_wall(double offset)
{
	navigator.turnController = normal_turn_controller;

	// Aligns with lines
	liftMid();
	mogoUp();
	navigatorTurnToAngle(&navigator, toRadians(-45+offset), 0.7, -0.2);
	delay(100);

	// Aligns with wall
	navigator.until_target = 43;
	navigatorDriveToDistance(&navigator, 15, toRadians(-45+offset), 0.6, 0);
	navigatorDriveToDistanceUntil(&navigator, 80, toRadians(-45+offset), 0.6, -0.3, UNTIL_FRONT_RIGHT_SONAR);
	delay(100);
	mogoDown();
	liftMid();
	navigatorTurnToAngle(&navigator, toRadians(0+offset), 0.6, -0.1);

	// Picks up mogo
	navigatorDriveToDistance(&navigator, 15, toRadians(0+offset), 1, 1);
	navigatorDriveToDistanceUntil(&navigator, 80, toRadians(0+offset), 1, -0.1, UNTIL_MOGO_FOUND);
	mogoUp();
	delay(500);

	// Scores mogo
	PSC_score_right_wall(offset);
}

void PSC_right_wall_with_loader(double offset)
{
	navigator.turnController = normal_turn_controller;

	// Aligns with lines
	liftMid();
	mogoUp();
	navigatorTurnToAngle(&navigator, toRadians(-45+offset), 0.7, -0.2);
	delay(100);

	// Aligns with wall
	navigator.until_target = 43;
	navigatorDriveToDistance(&navigator, 15, toRadians(-45+offset), 0.6, 0);
	navigatorDriveToDistanceUntil(&navigator, 80, toRadians(-45+offset), 0.6, -0.3, UNTIL_FRONT_RIGHT_SONAR);
	delay(100);
	mogoDown();
	liftMid();
	navigatorTurnToAngle(&navigator, toRadians(0+offset), 0.6, -0.1);

	// Picks up mogo
	navigatorDriveToDistance(&navigator, 15, toRadians(0+offset), 1, 1);
	navigatorDriveToDistanceUntil(&navigator, 80, toRadians(0+offset), 1, -0.1, UNTIL_MOGO_FOUND);
	mogoUp();
	delay(500);

	navigatorTurnToAngle(&navigator, toRadians(-18+offset), 0.6, -0.1);
	liftLoads();
	navigatorDriveToDistanceUntil(&navigator, -30, toRadians(-18+offset), 0.5, 0.1, UNTIL_RIGHT_LINE);
	//delay(200);
	navigatorDriveToDistance(&navigator, 1, toRadians(-18+offset), 0.5, -0.1);

	PSC_loader();

	navigatorDriveToDistance(&navigator, -30, toRadians(-18+offset), 0.7, -0.1);
	navigatorTurnToAngle(&navigator, toRadians(-135+offset), 0.6, -0.1);

	PSC_score_right_wall(offset);
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
	int driveL, driveR;
	int lift;
	int rollers;
	int mogo;

	//taskRunLoop(compControlTask, 100);
	taskRunLoop(odometryTask, 5);
	//taskRunLoop(debugTask, 100);
	taskRunLoop(mogoTask, 20);

	taskRunLoop(liftTask, 20);
	taskRunLoop(intakeTask, 50);
	taskRunLoop(encoderAnalogTask, 2);

  	//liftDown();
	//intakeIn();
	//Pose point = {.x = 20.0, .y = 20.0, .theta = 0.0};
	//navigatorTurnToPoint(&navigator, point, 1.0, 0.0);
	//navigatorDriveToPoint(&navigator, point, 1.0, 0.0);
	//delay(1000);
	//point = (Pose) {.x = 0.0, .y = 0.0, .theta = 0.0};
	//navigatorDriveToPoint(&navigator, point, -1.0, 0.0);

	//taskCreate(pidTuneTask, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);

	double targetAngle = 0.0;

	 straight_controller = pidControllerCreate(4, 0, 0);
	 naive_turn_controller = pidControllerCreate(1.5, 0, 0);
	 short_turn_controller = pidControllerCreate(2.4, 0, 0.05);
	 normal_turn_controller = pidControllerCreate(1.8, 0, 0.11);
	 normal_mogo_turn_controller = pidControllerCreate(2.5, 0, 0.25);
	 double_mogo_turn_controller = pidControllerCreate(3.0, 0, 0.4);
	 normal_right_turn_controller = pidControllerCreate(2.4, 0, 0.11);

    //front_left_sonar = ultrasonicInit(11, 9);
	//front_right_sonar = ultrasonicInit(12, 10);

	while (true) {
		//printf("encoderRoller = %d\n", encoder1WireCounts(encoderRoller));
		//printf("ultrasonicLeft = %d\n", ultrasonicGet(front_left_sonar));
	//	printf("ultrasonicRight = %d\n", ultrasonicGet(front_right_sonar));
		//int val = ultrasonicGet(front_left_sonar);
		//printf("val = %d \n", val);



		 int val = analogRead(leftBarDetect.port);
		 int val2 = analogRead(rightBarDetect.port);
		 int val3 = analogRead(backLine.port);
		  //int val = ultrasonicGet(front_left_sonar);
		 //int val2 = 0;//ultrasonicGet(front_right_sonar);
		 //printf("Bar = %d %d %d\n", val, val2, val3);

		  if (joystickGetDigital(1, 8, JOY_UP))
		  {
			  xsens_reset_heading(&xsens, 0, 0, 0);
			  odometryUseXsens(navigator.odometry);
			  //liftLoads();
			  //delay(2000);
			  //PSC_loader();
			  PSC_first_mogo();
			  PSC_double_mogo(45);
			  PSC_mogo_on_left_wall_single_cone(180);
			  //xsens_reset_heading(&xsens, 0, 0, 45);
			  //odometryUseXsens(navigator.odometry);
			  //PSC_right_wall_with_loader(180);
			  //PSC_mogo_on_left_wall_single_cone(180);
			  //PSC_mogo_on_left_wall(180, 0);
			  //PSC_mogo_on_right_wall(180);
			  //PSC_double_mogo(-135);
			  //PSC_mogo_on_left_wall(0, 0);

		  }

		  /*if (joystickGetDigital(1, 8, JOY_UP)) {
			  digitalWrite(mogo_tipper_port, HIGH);
		  }
		  else {
			  digitalWrite(mogo_tipper_port, LOW);
		  }

		  if (joystickGetDigital(1, 8, JOY_RIGHT)) {
			  digitalWrite(mogo_release_tipper_port, HIGH);
		  }
		  else {
			  digitalWrite(mogo_release_tipper_port, LOW);
		  }

		  if (joystickGetDigital(1, 8, JOY_DOWN)) {
			  mogoHoldUp();
		  }
		  else
		  	mogoUp();
			*/
		if (joystickGetDigital(1, 7, JOY_DOWN)) {
//			targetAngle += toRadians(90);
//			navigatorTurnToAngle(&navigator, targetAngle, 1.0, 0.0);
			xsens_reset_heading(&xsens, 0, 0, 0);




			//turn towards mogo 7

			printf("---Mogo7---");

		}
		odometryUseXsens(navigator.odometry);

		if (joystickGetDigital(1, 7, JOY_UP)) {


			navigator.straightController = straight_controller;
			navigator.turnController = normal_turn_controller;


		}
	/*	if (joystickGetDigital(1, 8, JOY_UP)) {
			mogoUp();
		} else if (joystickGetDigital(1, 8, JOY_RIGHT)) {
			mogoDown();
		}*/
		if (joystickGetDigital(1, 6, JOY_UP)) {
			liftPickupLoads();
		} else if (joystickGetDigital(1, 6, JOY_DOWN)) {
			liftDown();
		} else if (joystickGetDigital(1, 5, JOY_DOWN)) {
			liftUp();
			intakeIn();
		}

		int driveL = joystickGetAnalog(1, 3);
		int driveR = joystickGetAnalog(1, 2);
		driveSetPwm(&drive, driveL, driveR);
		//driveL = joystickGetAnalog(1, 3);
		//driveR = joystickGetAnalog(1, 2);
		//lift = joystickGetDigital(1, 5, JOY_UP) ? 127 : (joystickGetDigital(1, 5, JOY_DOWN) ? -127 : 0);
		//rollers = joystickGetDigital(1, 6, JOY_UP) ? 127 : (joystickGetDigital(1, 6, JOY_DOWN) ? -127 : 15);
		//mogo = joystickGetDigital(1, 8, JOY_UP) ? 127 : (joystickGetDigital(1, 8, JOY_RIGHT) ? -127 : -15);

		//driveSetPwm(&drive, driveL, driveR);
		//motorSetPwm(&motorLift, lift);
		//motorSetPwm(&motorRollers, rollers);
		//motorSetPwm(&motorMogo, mogo);
		//printf("intake pos: %d\n", getIntakePosition());

		delay(20);
	}
}
