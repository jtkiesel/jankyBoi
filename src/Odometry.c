#include "Odometry.h"

#include "API.h"
#include "EncoderWheel.h"
#include "log.h"
#include "Pose.h"
#include "xsens.h"

#include <math.h>

Odometry odometryCreate(EncoderWheel* encoderWheelL, EncoderWheel* encoderWheelR,
		EncoderWheel* encoderWheelM, struct XsensVex* xsens, double chassisWidth, Pose initialPose) {
	if (!encoderWheelL) {
		logError("odometryCreate", "encoderWheelL NULL");
		return (Odometry) {};
	}
	if (!encoderWheelR) {
		logError("odometryCreate", "encoderWheelR NULL");
		return (Odometry) {};
	}
	Odometry odometry = {.mutex = mutexCreate(), .encoderWheelL = encoderWheelL,
			.encoderWheelR = encoderWheelR, .encoderWheelM = encoderWheelM, .xsens = xsens,
			.chassisWidth = chassisWidth};
	odometrySetPose(&odometry, initialPose);
	return odometry;
}

void odometryDelete(Odometry* odometry) {
	if (!odometry) {
		logError("odometryDelete", "odometry NULL");
		return;
	}
	if (odometry->mutex) {
		mutexDelete(odometry->mutex);
	}
	odometry->mutex = NULL;
	odometry->encoderWheelL = NULL;
	odometry->encoderWheelR = NULL;
	odometry->encoderWheelM = NULL;
	odometry->chassisWidth = 0;
	odometry->pose = (Pose) {};
}

Pose odometryComputePose(Odometry* odometry) {
	if (!odometry) {
		logError("odometryComputePose", "odometry NULL");
		return (Pose) {};
	}
	mutexTake(odometry->mutex, 20);

	double dL = encoderWheelDistance(odometry->encoderWheelL) - odometry->lastL;
	double dR = encoderWheelDistance(odometry->encoderWheelR) - odometry->lastR;
	double dM = odometry->encoderWheelM ?
			(encoderWheelDistance(odometry->encoderWheelM) - odometry->lastM) : 0;
	double yaw = xsens_get_yaw(odometry->xsens) / 57.2958;

	odometry->lastL += dL;
	odometry->lastR += dR;
	odometry->lastM += dM;

	double dS = (dR + dL) / 2;
	Pose dPose = {.theta = yaw - odometry->pose.theta};  /*(dR - dL) / odometry->chassisWidth*/
	double avgTheta = odometry->pose.theta + dPose.theta / 2;

	dPose.x = dS * cos(avgTheta) + dM * sin(avgTheta);
	dPose.y = dS * sin(avgTheta) - dM * cos(avgTheta);

	poseAdd(&odometry->pose, dPose);

	mutexGive(odometry->mutex);

	return odometry->pose;
}

Pose odometryPose(const Odometry* odometry) {
	if (!odometry) {
		logError("odometryPose", "odometry NULL");
		return (Pose) {};
	}
	return odometry->pose;
}

void odometrySetPose(Odometry* odometry, Pose pose) {
	if (!odometry) {
		logError("odometrySetPose", "odometry NULL");
		return;
	}
	mutexTake(odometry->mutex, 20);

	odometry->lastL = encoderWheelDistance(odometry->encoderWheelL);
	odometry->lastR = encoderWheelDistance(odometry->encoderWheelR);
	odometry->lastM = encoderWheelDistance(odometry->encoderWheelM);

	odometry->pose = pose;

	mutexGive(odometry->mutex);
}
