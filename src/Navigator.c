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

void navigatorDriveAtAngle(Navigator* navigator, double angle, double power) {
	unsigned long t = micros();

	odometryUseXsens(navigator->odometry);

	double angleError = boundAngleNegPiToPi(angle - navigator->odometry->pose.theta);

	double anglePower = pidControllerComputeOutput(&navigator->straightController, angleError, t);

	double powerLeft = clampAbs(power - (anglePower / 2.0), 1.0);
	double powerRight = clampAbs(powerLeft + anglePower, 1.0);
	powerLeft = clampAbs(powerRight - anglePower, 1.0);

	driveSetPower(navigator->drive, powerLeft, powerRight);
}

void navigatorDriveForTime(Navigator* navigator, double leftPower, double rightPower, double time)
{
	driveSetPower(navigator->drive, leftPower, rightPower);

	delay(time);
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

void navigatorSmoothTurnToAngle(Navigator* navigator, double dir, double angle, double maxPower,
		double deadPower, double endPower) {
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
			if (dir < 0) {
				driveSetPower(navigator->drive, deadPower, power);
			// Right
			} else {
				driveSetPower(navigator->drive, -power, deadPower);
			}
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

void navigatorDriveToDistanceUntil(Navigator* navigator, double distance, double angle,
		double maxPower, double endPower, int until) {
	const double target = (encoderWheelDistance(navigator->odometry->encoderWheelL)
			+ encoderWheelDistance(navigator->odometry->encoderWheelR)) / 2.0 + distance;
	unsigned long t;
	double error;
	double power;

	if ((until & UNTIL_FRONT_LEFT_SONAR) != 0) {
		front_left_sonar = ultrasonicInit(11, 9);
	}

	if ((until & UNTIL_FRONT_RIGHT_SONAR) != 0) {
		front_right_sonar = ultrasonicInit(12, 10);
	}

	while (true) {
		t = micros();
		error = target - (encoderWheelDistance(navigator->odometry->encoderWheelL)
				+ encoderWheelDistance(navigator->odometry->encoderWheelR)) / 2.0;

		if (fabs(error) > navigator->driveDoneThreshold) {
			navigator->timestamp = 0;
			power = clampAbs(pidControllerComputeOutput(&navigator->driveController, error, t), maxPower);
			navigatorDriveAtAngle(navigator, angle, power);

			if ((until & UNTIL_LEFT_LINE) != 0 && lineSensorHasLine(&leftLine)) {
				break;
			}
			if ((until & UNTIL_RIGHT_LINE) != 0 && lineSensorHasLine(&rightLine)) {
				break;
			}
			if ((until & UNTIL_BACK_LINE) != 0 && lineSensorHasLine(&backLine)) {
				break;
			}
			if ((until & UNTIL_LEFT_BAR) != 0 && !lineSensorHasLine(&leftBarDetect)) {
				break;
			}
			if ((until & UNTIL_RIGHT_BAR) != 0 && !lineSensorHasLine(&rightBarDetect)) {
				break;
			}
			if ((until & UNTIL_MOGO_FOUND) != 0 && !lineSensorHasLine(&mogoDetect)) {
				break;
			}
			if ((until & UNTIL_FRONT_LEFT_SONAR) != 0) {
				int val = ultrasonicGet(front_left_sonar);
				static int good_count = 0;
				printf("val = %d \n", val);

				if (val > 0 && sgn(navigator->until_target) * val < navigator->until_target) {
					if ( navigator->until_target < 0 && val < -navigator->until_target+20) {
						good_count++;
					} else if (navigator->until_target > 0 && val > navigator->until_target-30 ){
						good_count++;
					}

				} else if (val == -1){ }
				else {
					good_count = 0;
				}
				if (good_count > 0) {
					good_count = 0;
					break;
				}
			}
			if ((until & UNTIL_FRONT_RIGHT_SONAR) != 0) {
				int val = ultrasonicGet(front_right_sonar);
				static int good_count_r = 0;
				printf("val = %d \n", val);

				if (val > 0 && sgn(navigator->until_target) * val < navigator->until_target) {
					if ( navigator->until_target < 0 && val < -navigator->until_target+20) {
						good_count_r++;
					} else if (navigator->until_target > 0 && val > navigator->until_target-30 ){
						good_count_r++;
					}
				} else {
					good_count_r = 0;
				}
				if (good_count_r > 0) {
					good_count_r = 0;
					break;
				}
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

	if ((until & UNTIL_FRONT_LEFT_SONAR) != 0) {
		ultrasonicShutdown(front_left_sonar);
	}

	if ((until & UNTIL_FRONT_RIGHT_SONAR) != 0) {
		ultrasonicShutdown(front_right_sonar);
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

bool navigatorAdaptiveDriveTowardsPoint(Navigator* navigator, Pose point, double maxPower, double endPower) {
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
		if (maxPower < 0.0) {
			// Drive backwards towards point.
			driveError *= -1.0;
			straightError += kPi;
		}
	} else {
		if (!navigator->isDeadReckoning) {
			navigator->isDeadReckoning = true;
			navigator->deadReckonReference = pose;
			navigator->deadReckonVector = (Vector) {.size = translation.size, .angle = pose.theta};
		}
		driveError = navigator->deadReckonVector.size -
				poseDistanceToPoint(pose, navigator->deadReckonReference);
		straightError = navigator->deadReckonVector.angle - pose.theta;
		if (maxPower < 0.0) {
			// Drive backwards towards point.
			driveError *= -1.0;
		}
	}
	straightError = boundAngleNegPiToPi(straightError);

	double drivePower = pidControllerComputeOutput(&navigator->driveController, driveError, t)
			+ endPower;
	double straightPower = pidControllerComputeOutput(&navigator->straightController, straightError, t);

	double leftPower = clampAbs(drivePower - (straightPower / 2.0), maxPower);
	double rightPower = clampAbs(leftPower + straightPower, maxPower);
	leftPower = clampAbs(rightPower - straightPower, maxPower);

	driveSetPower(navigator->drive, leftPower, rightPower);

	if (fabs(driveError) < navigator->driveDoneThreshold) {
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

bool navigatorAdaptiveTurnTowardsPoint(Navigator* navigator, Pose point, double maxPower, double endPower) {
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

	if (fabs(error) < navigator->turnDoneThreshold) {
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

void navigatorAdaptiveDriveToPoint(Navigator* navigator, Pose point, double maxPower, double endPower) {
	if (!navigator) {
		logError("navigatorDriveToPoint", "navigator NULL");
		return;
	}
	while (!navigatorAdaptiveDriveTowardsPoint(navigator, point, maxPower, endPower)) {
		delay(10);
	}
	return;
}

void navigatorAdaptiveDriveToPointUntil(Navigator* navigator, Pose point, double maxPower, double endPower, int until) {
	if (!navigator) {
		logError("navigatorDriveToPointUntil", "navigator NULL");
		return;
	}
	while (!navigatorAdaptiveDriveTowardsPoint(navigator, point, maxPower, endPower)) {
		if ((until & UNTIL_LEFT_LINE) != 0 && lineSensorHasLine(&leftLine)) {
			printf("Break on Left Line Sensor!!\n\n\n");
			break;
		}
		if ((until & UNTIL_RIGHT_LINE) != 0 && lineSensorHasLine(&rightLine)) {
			printf("Break on Right Line Sensor!!\n\n\n");
			break;
		}
		delay(10);
	}
	driveSetPowerAll(navigator->drive, endPower);
}

void navigatorAdaptiveTurnToPoint(Navigator* navigator, Pose point, double maxPower, double endPower) {
	if (!navigator) {
		logError("navigatorTurnToPoint", "navigator NULL");
		return;
	}
	while (!navigatorTurnTowardsPoint(navigator, point, maxPower, endPower)) {
		delay(10);
	}
}
