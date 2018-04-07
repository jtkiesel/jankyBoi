#include "Navigator.h"

#include "Drive.h"
#include "PidController.h"
#include "Pose.h"
#include "util.h"
#include "Vector.h"

#include <math.h>
#include <stdbool.h>

Navigator navigatorCreate(Drive* drive, Odometry* odometry, PidController driveController,
		PidController straightController, PidController turnController, double deadReckonRadius,
		double driveDoneThreshold, double turnDoneThreshold, unsigned long doneTime) {
	if (!drive) {
		printf("Error - navigatorCreate: drive NULL.\n");
		return (Navigator) {};
	}
	if (!odometry) {
		printf("Error - navigatorCreate: odometry NULL.\n");
		return (Navigator) {};
	}
	return (Navigator) {.drive = drive, .odometry = odometry,
			.driveController = driveController, .straightController = straightController,
			.turnController = turnController, .deadReckonRadius = deadReckonRadius,
			.driveDoneThreshold = driveDoneThreshold, .turnDoneThreshold = turnDoneThreshold,
			.doneTime = doneTime, .isDeadReckoning = false, .deadReckonReference = (Pose) {},
			.deadReckonVector = (Vector) {}, .timestamp = 0};
}

bool navigatorDriveToPoint(Navigator* navigator, Pose point, double maxPower, double endPower) {
	if (!navigator) {
		printf("Error - navigatorDriveToPoint: navigator NULL.\n");
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
	double drivePower = pidControllerComputeOutput(&navigator->driveController, driveError, t)
			+ endPower;
	double straightPower = pidControllerComputeOutput(&navigator->straightController, straightError, t);

	double leftPower = clampAbs(drivePower - (straightPower / 2.0), maxPower);
	double rightPower = clampAbs(leftPower + straightPower, maxPower);
	leftPower = clampAbs(rightPower - straightPower, maxPower);

	driveSetPower(navigator->drive, leftPower, rightPower);

	if (driveError < navigator->driveDoneThreshold) {
		if (endPower != 0) {
			return true;
		}
		if (navigator->timestamp == 0) {
			navigator->timestamp = millis();
		} else if (navigator->timestamp > navigator->doneTime) {
			navigator->timestamp = 0;
			return true;
		}
	} else {
		navigator->timestamp = 0;
	}
	return false;
}

bool navigatorTurnToFacePoint(Navigator* navigator, Pose point, double maxPower, double endPower) {
	if (!navigator) {
		printf("Error - navigatorTurnToFacePoint: navigator NULL.\n");
		return false;
	}
	unsigned long t = micros();
	Pose pose = odometryPose(navigator->odometry);

	double error = poseAngleToPoint(pose, point);
	double power = clampAbs(pidControllerComputeOutput(&navigator->turnController, error, t)
		+ endPower, maxPower);

	driveSetPower(navigator->drive, -power, power);

	if (error < navigator->turnDoneThreshold) {
		if (endPower != 0) {
			return true;
		}
		if (navigator->timestamp == 0) {
			navigator->timestamp = millis();
		} else if (navigator->timestamp > navigator->doneTime) {
			navigator->timestamp = 0;
			return true;
		}
	} else {
		navigator->timestamp = 0;
	}
	return false;
}
