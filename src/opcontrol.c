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
#include "Odometry.h"
#include "Pose.h"
#include "xsens.h"

#include <math.h>

void odometryTask() {
	odometryComputePose(&odometry);
}

void debugTask() {
	printf("(%.3f x, %.3f y, %f theta)\n", odometry.pose.x, odometry.pose.y, odometry.pose.theta);
	//printf("xsens yaw: %.3f\n", xsens_get_yaw(&xsens));
	//const Vector translation = poseTranslationToPoint(odometry.pose, (Pose) {.x = 30.0, .y = 0.0, .theta = 0.0});//odometryPose(&odometry);
	//printf("translation: (%.3f size, %f angle)\n", translation.size, translation.angle);
}

typedef enum MogoState {
	MogoUp,
	MogoDown
} MogoState;

MogoState mogoState = MogoUp;

void mogoUp() {
	mogoState = MogoUp;
}

void mogoDown() {
	mogoState = MogoDown;
}

int getMogoPosition() {
	int value;
	imeGet(imeMogo, &value);
	return value;
}

void mogoTask() {
	if (mogoState == MogoUp) {
		if (getMogoPosition() < 1000) {
			motorSetPower(&motorMogo, 1.0);
		} else {
			motorSetPower(&motorMogo, 0.05);
		}
	} else {
		while (getMogoPosition() > 100) {
			motorSetPower(&motorMogo, -1.0);
			delay(20);
		}
		motorSetPower(&motorMogo, -0.05);
	}
}

typedef enum LiftState {
	LiftUp,
	LiftDown,
	LiftMid
} LiftState;

LiftState liftState = LiftUp;

void liftUp() {
	liftState = LiftUp;
}

void liftDown() {
	liftState = LiftDown;
}

int getLiftPosition() {
	int value;
	imeGet(imeLift, &value);
	return value;
}

void liftTask() {

	int liftPosition = getLiftPosition();
	double error = 0;
	double hold = 0;

	if (liftState == LiftUp) {
		error = 0 - liftPosition;
		if (abs(error) < 20) hold = 0.05;
	} else if (liftState == LiftMid){
		error = -175 - liftPosition;
		//if (abs(error) < 20) hold = 0.05;
	}
	else {
		error = -950 - liftPosition;
		if (abs(error) < 20) hold = -0.05;
	}

	double pid_output = pidControllerComputeOutput(&liftController, error, 1);

	if (hold != 0)
		motorSetPower(&motorLift, hold);
	else
	  motorSetPower(&motorLift, pid_output);
	//printf("lift position %d, output = %f\n", liftPosition, pid_output);
}

int getIntakePosition()
{
	int value;
	imeGet(imeRollers, &value);
	return value;
}

typedef enum IntakeState {
	IntakeIn,
	IntakeOut,
	IntakeNone
} IntakeState;

IntakeState intakeState = IntakeNone;

void intakeIn()
{
	intakeState = IntakeIn;
}

void intakeOut()
{
	intakeState = IntakeOut;
}

void intakeTask()
{
	static int lastPosition = 0;
	static int velocity_zero_counter = 0;

	int intakePosition = getIntakePosition();
	int intakeVelocity = intakePosition - lastPosition;

	if (intakeState == IntakeNone)
	{
		motorSetPower(&motorRollers, 0);
	}
	else if (intakeState == IntakeIn)
	{
		if (intakeVelocity < 2) velocity_zero_counter++;
		else velocity_zero_counter = 0;

		if (velocity_zero_counter > 3) motorSetPower(&motorRollers, 0.05);
		else motorSetPower(&motorRollers, 1.0);
	}
	else if (intakeState == IntakeOut)
	{
		motorSetPower(&motorRollers, -1.0);
	}

	//printf("Intake Velocity = %d\n", intakeVelocity);

	lastPosition = intakePosition;
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

	//taskRunLoop(odometryTask, 2);
	taskRunLoop(debugTask, 100);
	//taskRunLoop(mogoTask, 20);

	taskRunLoop(liftTask, 20);
	taskRunLoop(intakeTask, 50);

  liftState = LiftDown;
	intakeState = IntakeIn;
	//Pose point = {.x = 20.0, .y = 20.0, .theta = 0.0};
	//navigatorTurnToPoint(&navigator, point, 1.0, 0.0);
	//navigatorDriveToPoint(&navigator, point, 1.0, 0.0);
	//delay(1000);
	//point = (Pose) {.x = 0.0, .y = 0.0, .theta = 0.0};
	//navigatorDriveToPoint(&navigator, point, -1.0, 0.0);

	while (true) {
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
