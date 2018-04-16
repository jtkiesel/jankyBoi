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

void navigatorSmoothTurnToAngle(Navigator* navigator, double dir, double angle, double maxPower, double deadPower, double endPower)
{
	unsigned long t;
	double error;
	double power;

	while (true) {
		t = micros();
		error = boundAngleNegPiToPi(angle - navigator->odometry->pose.theta);

		if (fabs(error) > navigator->turnDoneThreshold) {
			navigator->timestamp = 0;
			power = clampAbs(pidControllerComputeOutput(&navigator->turnController, error, t), maxPower);

			// Left
			if (dir < 0)
				driveSetPower(navigator->drive, deadPower, power);
			// Right
			else
				driveSetPower(navigator->drive, -power, deadPower);
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

void navigatorDriveToDistanceUntil(Navigator* navigator, double distance, double angle, double maxPower, double endPower, int until) {
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

			if ((until & UNTIL_FRONT_LEFT_SONAR) != 0)
			{
				int val = ultrasonicGet(front_left_sonar);
				static good_count = 0;
				printf("val = %d \n", val);

				if (val > 0 && val < navigator->until_target)
					good_count++;
				else good_count = 0;

				if (good_count > 1) break;
			}

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
	driveSetPowerAll(navigator->drive, endPower);
}

void navigatorTurnToAngle(Navigator* navigator, double angle, double maxPower, double endPower) {
	unsigned long t;
	double error;
	double power;

	while (true) {
		t = micros();
		error = boundAngleNegPiToPi(angle - navigator->odometry->pose.theta);

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

void navigatorDriveToPoint(Navigator* navigator, Pose point, double maxPower, double endPower) {
	double dx = point.x - navigator->odometry->pose.x;
	double dy = point.y - navigator->odometry->pose.y;
	double distance = hypot(dy, dx);
	double angle = atan2(dy, dx);
	if (maxPower < 0.0) {
		distance *= -1.0;
		angle += kPi;
	}
	navigatorDriveToDistance(navigator, distance, angle, maxPower, endPower);
}

void navigatorTurnToPoint(Navigator* navigator, Pose point, double maxPower, double endPower) {
	double dx = point.x - navigator->odometry->pose.x;
	double dy = point.y - navigator->odometry->pose.y;
	double angle = atan2(dy, dx);
	if (maxPower < 0.0) {
		angle += kPi;
	}
	navigatorTurnToAngle(navigator, angle, maxPower, endPower);
}
