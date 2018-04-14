#include "Navigator.h"

#include "Drive.h"
#include "log.h"
#include "PidController.h"
#include "Pose.h"
#include "util.h"
#include "Vector.h"
#include "globals.h"

#include <math.h>
#include <stdbool.h>

Navigator navigatorCreate(Drive* drive, Odometry* odometry, PidController driveController,
		PidController straightController, PidController turnController, double deadReckonRadius,
		double driveDoneThreshold, double turnDoneThreshold, unsigned long doneTime) {
	if (!drive) {
		logError("navigatorCreate", "drive NULL");
		return (Navigator) {};
	}
	if (!odometry) {
		logError("navigatorCreate", "odometry NULL");
		return (Navigator) {};
	}
	return (Navigator) {.drive = drive, .odometry = odometry,
			.driveController = driveController, .straightController = straightController,
			.turnController = turnController, .deadReckonRadius = deadReckonRadius,
			.driveDoneThreshold = driveDoneThreshold, .turnDoneThreshold = turnDoneThreshold,
			.doneTime = doneTime, .isDeadReckoning = false, .deadReckonReference = (Pose) {},
			.deadReckonVector = (Vector) {}, .timestamp = 0};
}

bool navigatorDriveTowardsPoint(Navigator* navigator, Pose point, double maxPower, double endPower) {
	if (!navigator) {
		logError("navigatorDriveTowardsPoint", "navigator NULL");
		return false;
	}
	unsigned long t = micros();

	Pose pose = odometryPose(navigator->odometry);
	Vector translation = poseTranslationToPoint(pose, point);

	double driveError;
	double straightError;
	if (translation.size > navigator->deadReckonRadius) {
		navigator->isDeadReckoning = false;
		driveError = translation.size;
		straightError = translation.angle;
	} else {
		if (!navigator->isDeadReckoning) {
			navigator->isDeadReckoning = true;
			navigator->deadReckonReference = pose;
			navigator->deadReckonVector = (Vector) {.size = translation.size, .angle = pose.theta};
		}
		driveError = navigator->deadReckonVector.size -
				poseDistanceToPoint(pose, navigator->deadReckonReference);
		straightError = navigator->deadReckonVector.angle - pose.theta;
	}
	if (maxPower < 0.0) {
		// Drive backwards towards point.
		driveError *= -1.0;
		straightError += kPi;
	}
	straightError = boundAngleNegPiToPi(straightError);

	double drivePower = pidControllerComputeOutput(&navigator->driveController, driveError, t)
			+ endPower;
	double straightPower = pidControllerComputeOutput(&navigator->straightController, straightError, t);

	double leftPower = clampAbs(drivePower - (straightPower / 2.0), maxPower);
	double rightPower = clampAbs(leftPower + straightPower, maxPower);
	leftPower = clampAbs(rightPower - straightPower, maxPower);

	driveSetPower(navigator->drive, leftPower, rightPower);

	if (driveError < navigator->driveDoneThreshold) {
		if (fabs(endPower) > 0.000001) {
			driveSetPowerAll(navigator->drive, endPower);
			return true;
		}
		if (navigator->timestamp == 0) {
			navigator->timestamp = millis();
		} else if ((millis() - navigator->timestamp) > navigator->doneTime) {
			navigator->timestamp = 0;
			driveSetPowerAll(navigator->drive, endPower);
			return true;
		}
	} else {
		navigator->timestamp = 0;
	}
	return false;
}

bool navigatorTurnTowardsPoint(Navigator* navigator, Pose point, double maxPower, double endPower) {
	if (!navigator) {
		logError("navigatorTurnTowardsPoint", "navigator NULL");
		return false;
	}
	unsigned long t = micros();
	Pose pose = odometryPose(navigator->odometry);

	double error = poseAngleToPoint(pose, point);
	if (maxPower < 0.0) {
		// Turn backside towards point.
		error = boundAngleNegPiToPi(error + kPi);
	}
	double power = clampAbs(pidControllerComputeOutput(&navigator->turnController, error, t)
			+ endPower, maxPower);

	driveSetPower(navigator->drive, -power, power);

	if (error < navigator->turnDoneThreshold) {
		if (fabs(endPower) > 0.000001) {
			print("endPower != 0\n");
			driveSetPower(navigator->drive, -endPower, endPower);
			return true;
		}
		if (navigator->timestamp == 0) {
			navigator->timestamp = millis();
		} else if ((millis() - navigator->timestamp) > navigator->doneTime) {
			navigator->timestamp = 0;
			driveSetPower(navigator->drive, -endPower, endPower);
			return true;
		}
	} else {
		navigator->timestamp = 0;
	}
	return false;
}

bool navigatorDriveToPoint(Navigator* navigator, Pose point, double maxPower, double endPower) {
	if (!navigator) {
		logError("navigatorDriveToPoint", "navigator NULL");
		return false;
	}
	while (!navigatorDriveTowardsPoint(navigator, point, maxPower, endPower)) {
		delay(2);
	}
	return true;
}

bool navigateDriveToPointUntil(Navigator* navigator, Pose point, double maxPower, double endPower, int until)
{
	if (!navigator) {
		logError("navigatorDriveToPointUntil", "navigator NULL");
		return false;
	}

	while (!navigatorDriveTowardsPoint(navigator, point, maxPower, endPower)) {

		if ((until & UNTIL_LEFT_LINE) != 0)
		{
			if (lineSensorHasLine(&leftLine)) break;
		}

		if ((until & UNTIL_RIGHT_LINE) != 0)
		{
			if (lineSensorHasLine(&rightLine)) break;
		}

		if ((until & UNTIL_BACK_LINE) != 0)
		{
			if (lineSensorHasLine(&backLine)) break;
		}

		delay(2);
	}
	return true;
}

bool navigatorTurnToPoint(Navigator* navigator, Pose point, double maxPower, double endPower) {
	if (!navigator) {
		logError("navigatorTurnToPoint", "navigator NULL");
		return false;
	}
	while (!navigatorTurnTowardsPoint(navigator, point, maxPower, endPower)) {
		delay(2);
	}
	return true;
}
