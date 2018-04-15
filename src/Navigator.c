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
		PidController straightController, PidController turnController, double driveDoneThreshold,
		double turnDoneThreshold, unsigned long doneTime) {
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
			.turnController = turnController, .driveDoneThreshold = driveDoneThreshold,
			.turnDoneThreshold = turnDoneThreshold, .doneTime = doneTime, .timestamp = 0};
}

void navigatorDriveAtAngle(Navigator* navigator, double angle, double power) {
	unsigned long t = micros();
	double angleError = boundAngleNegPiToPi(angle - navigator->odometry->pose.theta);

	double anglePower = pidControllerComputeOutput(&navigator->straightController, angleError, t);

	double powerLeft = clampAbs(power - (anglePower / 2.0), 1.0);
	double powerRight = clampAbs(powerLeft + anglePower, 1.0);
	powerLeft = clampAbs(powerRight - anglePower, 1.0);

	driveSetPower(navigator->drive, powerLeft, powerRight);
}

void navigatorDriveToDistance(Navigator* navigator, double distance, double angle, double maxPower, double endPower) {
	const double target = (encoderWheelDistance(navigator->odometry->encoderWheelL)
			+ encoderWheelDistance(navigator->odometry->encoderWheelR)) / 2.0 + distance;
	unsigned long t;
	double error;
	double power;

	while (true) {
		t = micros();
		error = target - (encoderWheelDistance(navigator->odometry->encoderWheelL)
				+ encoderWheelDistance(navigator->odometry->encoderWheelR)) / 2.0;

		if (fabs(error) > navigator->driveDoneThreshold) {
			navigator->timestamp = 0;
			power = clampAbs(pidControllerComputeOutput(&navigator->driveController, error, t), maxPower);
			navigatorDriveAtAngle(navigator, angle, power);
		} else {
			if (fabs(endPower) > 0.000001) {
				driveSetPowerAll(navigator->drive, endPower);
				return;
			}
			if (navigator->timestamp == 0) {
				navigator->timestamp = millis();
			} else if ((millis() - navigator->timestamp) > navigator->doneTime) {
				navigator->timestamp = 0;
				driveSetPowerAll(navigator->drive, endPower);
				return;
			}
		}
		delay(10);
	}
}

void navigatorTurnToAngle(Navigator* navigator, double angle, double maxPower, double endPower) {
	unsigned long t;
	double error;
	double power;

	while (true) {
		t = micros();
		error = boundAngle0To2Pi(angle - navigator->odometry->pose.theta);

		if (fabs(error) > navigator->turnDoneThreshold) {
			navigator->timestamp = 0;
			power = clampAbs(pidControllerComputeOutput(&navigator->turnController, error, t), maxPower);
			driveSetPower(navigator->drive, -power, power);
		} else {
			if (fabs(endPower) > 0.000001) {
				driveSetPower(navigator->drive, -endPower, endPower);
				return;
			}
			if (navigator->timestamp == 0) {
				navigator->timestamp = millis();
			} else if ((millis() - navigator->timestamp) > navigator->doneTime) {
				navigator->timestamp = 0;
				driveSetPower(navigator->drive, -endPower, endPower);
				return;
			}
		}
		delay(10);
	}
}
