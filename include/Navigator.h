#ifndef NAVIGATOR_H_
#define NAVIGATOR_H_

#include "Drive.h"
#include "Odometry.h"
#include "PidController.h"
#include "Pose.h"
#include "Vector.h"

#include <stdbool.h>

typedef struct Navigator {
	Drive* drive;
	Odometry* odometry;
	PidController driveController;
	PidController straightController;
	PidController turnController;
	double deadReckonRadius;
	double driveDoneThreshold;
	double turnDoneThreshold;
	unsigned long doneTime;
	bool isDeadReckoning;
	Pose deadReckonReference;
	Vector deadReckonVector;
	unsigned long timestamp;
} Navigator;

Navigator navigatorCreate(Drive* drive, Odometry* odometry, PidController driveController,
		PidController straightController, PidController turnController, double deadReckonRadius,
		double driveDoneThreshold, double turnDoneThreshold, unsigned long doneTime);

bool navigatorDriveTowardsPoint(Navigator* navigator, Pose point, double maxPower, double endPower);

bool navigatorTurnTowardsPoint(Navigator* navigator, Pose point, double maxPower, double endPower);

bool navigatorDriveToPoint(Navigator* navigator, Pose point, double maxPower, double endPower);

bool navigatorTurnToPoint(Navigator* navigator, Pose point, double maxPower, double endPower);

#endif  // NAVIGATOR_H_
