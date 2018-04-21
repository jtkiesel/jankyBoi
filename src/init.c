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
#include "EncoderAnalog.h"
#include "Encoder1Wire.h"
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
	motorLift = motorCreate(4, false);
	motorRollers = motorCreate(5, false);
	// Motor port 6 empty.
	motorMogo = motorCreate(7, false);
	motorDriveR = motorCreate(8, true);
	motorDriveR2 = motorCreate(9, true);

	/**
	 * Sensors.
	 */
	encoderDriveL = encoderInit(5, 6, true);
	encoderDriveR = encoderInit(1, 2, false);
	encoderDriveM = encoderInit(3, 4, false);
	encoderLift = encoderInit(7, 8, false);

	encoderRoller = encoderAnalogCreate(7);


	int line_toggle = 1200;
	leftLine = lineSensorCreate(1, line_toggle);
	rightLine = lineSensorCreate(2, line_toggle);

	leftBarDetect = lineSensorCreate(3, 1050);
	rightBarDetect = lineSensorCreate(4, 1150);

	/**
	 * Objects.
	 */
	encoderWheelL = encoderWheelCreate(encoderDriveL, 360, 3.232, 5, 1);
	encoderWheelR = encoderWheelCreate(encoderDriveR, 360, 3.232, 5, 1);
	encoderWheelM = encoderWheelCreate(encoderDriveM, 360, 3.232, 5, 1);

	print("Calibrating Xsens.\n");
	xsens_init(&xsens, uart1, 38400);
	logger_set_level(&xsens.log, LOGLEVEL_ERROR);
	logger_set_level(logger_get_global_log(), LOGLEVEL_ERROR);
	xsens_start_task(&xsens);
	xsens_calibrate(&xsens, 100);
	print("Done calibrating Xsens.\n");

	//const Pose initialPose = poseCreate(72, 24, 0);
	const Pose initialPose = poseCreate(0, 0, 0);
	odometry = odometryCreate(&encoderWheelL, &encoderWheelR, &encoderWheelM, &xsens, 7.90, initialPose);

	drive = driveCreate(&motorDriveL, &motorDriveR, &motorDriveL2, &motorDriveR2);
	const PidController drivePidController = pidControllerCreate(0.15, 0.0, 0.0);
	const PidController straightPidController = pidControllerCreate(2, 0, 0);
	const PidController turnPidController = pidControllerCreate(3.4, 0, 0.26);
	navigator = navigatorCreate(&drive, &odometry, drivePidController, straightPidController,
			turnPidController, 10, 0.5, 0.1, 0);

	liftController = pidControllerCreate(0.01, 0.0, 0.0);
}
