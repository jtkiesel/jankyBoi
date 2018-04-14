/** @file auto.c
 * @brief File for autonomous code
 *
 * This file should contain the user autonomous() function and any functions related to it.
 *
 * Any copyright is dedicated to the Public Domain.
 * http://creativecommons.org/publicdomain/zero/1.0/
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "API.h"
#include "globals.h"
#include "main.h"
#include "Motor.h"
#include "Navigator.h"
#include "Odometry.h"
#include "Pose.h"
#include "xsens.h"

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

//const int kMogoLimitThreshold = 60;

void mogoTask() {
	if (mogoState == MogoUp) {
		if (getMogoPosition() < (0 - 60)) {
			motorSetPower(&motorMogo, 1.0);
		} else {
			motorSetPower(&motorMogo, 0.05);
		}
	} else {
		if (getMogoPosition() > (-625 + 60)) {
			motorSetPower(&motorMogo, -1.0);
		} else {
			motorSetPower(&motorMogo, -0.05);
		}
	}
}

void waitUntilMogo() {
	if (mogoState == MogoUp) {
		while (getMogoPosition() < (0 - 60)) {
			delay(20);
		}
	} else {
		while (getMogoPosition() > (-625 + 60)) {
			delay(20);
		}
	}
}

typedef enum LiftState {
	LiftUp,
	LiftDown
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
	if (liftState == LiftUp) {
		if (getLiftPosition() < 1000) {
			motorSetPower(&motorLift, 1.0);
		} else {
			motorSetPwm(&motorLift, 10);
		}
	} else {
		if (getLiftPosition() > 100) {
			motorSetPower(&motorLift, -1.0);
		} else {
			motorSetPwm(&motorLift, -20);
		}
	}
}

void odometryTask() {
	odometryComputePose(&odometry);
}

void debugTask() {
	printf("(%.3f x, %.3f y, %f theta)\n", odometry.pose.x, odometry.pose.y, odometry.pose.theta);
	//printf("xsens yaw: %.3f\n", xsens_get_yaw(&xsens));
	//const Vector translation = poseTranslationToPoint(odometry.pose, (Pose) {.x = 30.0, .y = 0.0, .theta = 0.0});//odometryPose(&odometry);
	//printf("translation: (%.3f size, %f angle)\n", translation.size, translation.angle);
	printf("mogo: %d\n", getMogoPosition());
	printf("left line: %d\n", analogRead(1));
}

/*
 * Runs the user autonomous code. This function will be started in its own task with the default
 * priority and stack size whenever the robot is enabled via the Field Management System or the
 * VEX Competition Switch in the autonomous mode. If the robot is disabled or communications is
 * lost, the autonomous task will be stopped by the kernel. Re-enabling the robot will restart
 * the task, not re-start it from where it left off.
 *
 * Code running in the autonomous task cannot access information from the VEX Joystick. However,
 * the autonomous function can be invoked from another task if a VEX Competition Switch is not
 * available, and it can access joystick information if called in this way.
 *
 * The autonomous task may exit, unlike operatorControl() which should never exit. If it does
 * so, the robot will await a switch to another mode or disable/enable cycle.
 */
void autonomous() {
	taskRunLoop(odometryTask, 2);
	taskRunLoop(debugTask, 100);
	taskRunLoop(mogoTask, 20);

	mogoDown();
	waitUntilMogo();
	navigatorDriveToPoint(&navigator, (Pose) {.x = 5, .y = 0, .theta = 0}, 0.5, 0.5);
	navigatorDriveToPointUntil(&navigator, (Pose) {.x = 40, .y = 0, .theta = 0}, 0.5, 0.5, UNTIL_LEFT_LINE);
	odometrySetPose(&odometry, (Pose) {.x = 30, .y = odometry.pose.y, .theta = odometry.pose.theta});
	navigatorDriveToPoint(&navigator, (Pose) {.x = 45, .y = 0, .theta = 0}, 1.0, -0.05);
	mogoUp();
	delay(1500);
	navigatorDriveToPointUntil(&navigator, (Pose) {.x = 20, .y = 0, .theta = 0}, -0.5, -0.5, UNTIL_LEFT_LINE);
	odometrySetPose(&odometry, (Pose) {.x = 30, .y = odometry.pose.y, .theta = odometry.pose.theta});
	navigatorDriveToPoint(&navigator, (Pose) {.x = 12, .y = 0, .theta = 0}, -0.5, 0.1);
	delay(250);
	navigatorTurnToPoint(&navigator, (Pose) {.x = 0, .y = 24, .theta = 0}, 0.8, 0.8);
	navigatorTurnToPoint(&navigator, (Pose) {.x = -10, .y = -10, .theta = 0}, 0.8, 0);
	unsigned long timestamp = millis();
	while (millis() - timestamp < 500) {
		navigatorDriveTowardsPoint(&navigator, (Pose) {.x = -10, .y = -10, .theta = 0}, 0.5, 0);
		delay(20);
	}
	driveSetPowerAll(&drive, 0);
	mogoDown();
	delay(500);
	navigatorDriveToPoint(&navigator, (Pose) {.x = 68, .y = 31, .theta = 0}, -0.5, 0.05);
	navigatorTurnToPoint(&navigator, (Pose) {.x = 30, .y = 30, .theta = 0}, 0.8, -0.1);
	navigatorDriveToPoint(&navigator, (Pose) {.x = 30, .y = 30, .theta = 0}, 0.5, -0.05);
	mogoUp();
	delay(1500);
}
