/** @file init.c
 * @brief File for initialization code
 *
 * This file should contain the user initialize() function and any functions related to it.
 *
 * Any copyright is dedicated to the Public Domain.
 * http://creativecommons.org/publicdomain/zero/1.0/
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"

#include "API.h"
#include "Drive.h"
#include "EncoderWheel.h"
#include "globals.h"
#include "Motor.h"
#include "Navigator.h"
#include "Odometry.h"
#include "PidController.h"
#include "Pose.h"
#include "xsens.h"

/*
 * Runs pre-initialization code. This function will be started in kernel mode one time while the
 * VEX Cortex is starting up. As the scheduler is still paused, most API functions will fail.
 *
 * The purpose of this function is solely to set the default pin modes (pinMode()) and port
 * states (digitalWrite()) of limit switches, push buttons, and solenoids. It can also safely
 * configure a UART port (usartOpen()) but cannot set up an LCD (lcdInit()).
 */
void initializeIO() {
}

/*
 * Runs user initialization code. This function will be started in its own task with the default
 * priority and stack size once when the robot is starting up. It is possible that the VEXnet
 * communication link may not be fully established at this time, so reading from the VEX
 * Joystick may fail.
 *
 * This function should initialize most sensors (gyro, encoders, ultrasonics), LCDs, global
 * variables, and IMEs.
 *
 * This function must exit relatively promptly, or the operatorControl() and autonomous() tasks
 * will not start. An autonomous mode selection menu like the pre_auton() in other environments
 * can be implemented in this task if desired.
 */
void initialize() {
	/**
	 * Motors.
	 */
	motorDriveL = motorCreate(2, false);
	motorDriveL2 = motorCreate(3, false);
	motorRollers = motorCreate(4, false);
	motorLift = motorCreate(5, false);
	// Motor port 6 empty.
	motorMogo = motorCreate(7, true);
	motorDriveR = motorCreate(8, true);
	motorDriveR2 = motorCreate(9, true);

	/**
	 * Sensors.
	 */
	encoderDriveL = encoderInit(5, 6, true);
	encoderDriveR = encoderInit(1, 2, false);
	encoderDriveM = encoderInit(3, 4, false);

	int line_toggle = 1200;
	leftLine = lineSensorCreate(1, line_toggle);
	rightLine = lineSensorCreate(2, line_toggle);
	backLine = lineSensorCreate(3, line_toggle);

	/**
	 * Objects.
	 */
	encoderWheelL = encoderWheelCreate(encoderDriveL, 360, 3.232, 5, 1);
	encoderWheelR = encoderWheelCreate(encoderDriveR, 360, 3.232, 5, 1);
	encoderWheelM = encoderWheelCreate(encoderDriveM, 360, 3.232, 5, 1);

	xsens_init(&xsens, uart1, 38400);
	logger_set_level(&xsens.log, LOGLEVEL_ERROR);
	logger_set_level(logger_get_global_log(), LOGLEVEL_ERROR);
	xsens_start_task(&xsens);
	xsens_calibrate(&xsens, 100);

	const Pose initialPose = poseCreate(0, 0, 0);
	odometry = odometryCreate(&encoderWheelL, &encoderWheelR, &encoderWheelM, &xsens, 7.99011, initialPose);

	drive = driveCreate(&motorDriveL, &motorDriveR, &motorDriveL2, &motorDriveR2);
	const PidController drivePidController = pidControllerCreate(0.15, 0.0, 0.0);
	const PidController straightPidController = pidControllerCreate(1.5, 0.0, 0.0);
	const PidController turnPidController = pidControllerCreate(3.0, 0.0, 230000.0);
	navigator = navigatorCreate(&drive, &odometry, drivePidController, straightPidController,
			turnPidController, 10.0, 0.5, 0.05, 1000);
}
