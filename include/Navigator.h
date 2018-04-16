#ifndef NAVIGATOR_H_
#define NAVIGATOR_H_

#include "Drive.h"
#include "Odometry.h"
#include "PidController.h"
#include "Pose.h"
#include "Vector.h"
#include "LineSensor.h"

#include <stdbool.h>

#define UNTIL_LEFT_LINE 0x01
#define UNTIL_RIGHT_LINE 0x02
#define UNTIL_BACK_LINE 0x04
#define UNTIL_BACK_SONAR 0x08
#define UNTIL_FRONT_LEFT_SONAR 0x10
#define UNTIL_FRONT_RIGHT_SONAR 0x20


#define SMOOTH_TURN_LEFT -1
#define SMOOTH_TURN_RIGHT 1

typedef struct Navigator {
	Drive* drive;
	Odometry* odometry;
	PidController driveController;
	PidController straightController;
	PidController turnController;
	double driveDoneThreshold;
	double turnDoneThreshold;
	unsigned long doneTime;
	unsigned long timestamp;

	double until_target;
} Navigator;

Navigator navigatorCreate(Drive* drive, Odometry* odometry, PidController driveController,
		PidController straightController, PidController turnController, double driveDoneThreshold,
		double turnDoneThreshold, unsigned long doneTime);

void navigatorDriveAtAngle(Navigator* navigator, double angle, double power);

bool navigatorTurnTowardsPoint(Navigator* navigator, Pose point, double maxPower, double endPower);
void navigatorDriveToDistance(Navigator* navigator, double distance, double angle, double maxPower, double endPower);
void navigatorTurnToAngle(Navigator* navigator, double angle, double maxPower, double endPower) ;
void navigatorDriveToDistanceUntil(Navigator* navigator, double distance, double angle, double maxPower, double endPower, int until);
void navigatorSmoothTurnToAngle(Navigator* navigator, double dir, double angle, double maxPower, double deadPower, double endPower);
void navigatorTurnToAngle(Navigator* navigator, double angle, double maxPower, double endPower);

void navigatorDriveToPoint(Navigator* navigator, Pose point, double maxPower, double endPower);

void navigatorTurnToPoint(Navigator* navigator, Pose point, double maxPower, double endPower);

void navigatorDriveToPointUntil(Navigator* navigator, Pose point, double maxPower, double endPower, int until);

#endif  // NAVIGATOR_H_
