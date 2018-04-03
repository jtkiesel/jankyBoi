#include "Odometry.h"

#include <math.h>

#include "API.h"
#include "EncoderWheel.h"
#include "Pose.h"

Odometry odometryCreate(EncoderWheel* encoderWheelL, EncoderWheel* encoderWheelR,
		EncoderWheel* encoderWheelM, double chassisWidth, Pose initialPose) {
	if (!encoderWheelL) {
		printf("Error - odometryCreate: encoderWheelL NULL.\n");
		return (Odometry) {};
	}
	if (!encoderWheelR) {
		printf("Error - odometryCreate: encoderWheelR NULL.\n");
		return (Odometry) {};
	}
	Odometry odometry = {.mutex = mutexCreate(), .encoderWheelL = encoderWheelL,
			.encoderWheelR = encoderWheelR, .encoderWheelM = encoderWheelM,
			.chassisWidth = chassisWidth};
	odometrySetPose(&odometry, initialPose);
	return odometry;
}

void odometryDelete(Odometry* odometry) {
	if (!odometry) {
		printf("Error - odometryDelete: odometry NULL.\n");
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
		printf("Error - odometryComputePose: odometry NULL.\n");
		return (Pose) {};
	}
	mutexTake(odometry->mutex, 20);

	double dL = encoderWheelDistance(odometry->encoderWheelL) - odometry->lastL;
	double dR = encoderWheelDistance(odometry->encoderWheelR) - odometry->lastR;
	double dM = encoderWheelDistance(odometry->encoderWheelM) - odometry->lastM;

	odometry->lastL += dL;
	odometry->lastR += dR;
	odometry->lastM += dM;

	double dS = (dR + dL) / 2;
	Pose dPose = {.theta = (dR - dL) / odometry->chassisWidth};
	double avgTheta = odometry->pose.theta + dPose.theta / 2;

	dPose.x = dS * cos(avgTheta) + dM * sin(avgTheta);
	dPose.y = dS * sin(avgTheta) - dM * cos(avgTheta);

	poseAdd(&odometry->pose, dPose);

	mutexGive(odometry->mutex);

	return odometry->pose;
}

Pose odometryPose(const Odometry* odometry) {
	if (!odometry) {
		printf("Error - odometryPose: odometry NULL.\n");
		return (Pose) {};
	}
	return odometry->pose;
}

void odometrySetPose(Odometry* odometry, Pose pose) {
	if (!odometry) {
		printf("Error - odometrySetPose: odometry NULL.\n");
		return;
	}
	mutexTake(odometry->mutex, 20);

	odometry->lastL = encoderWheelDistance(odometry->encoderWheelL);
	odometry->lastR = encoderWheelDistance(odometry->encoderWheelR);
	odometry->lastM = encoderWheelDistance(odometry->encoderWheelM);

	odometry->pose = pose;

	mutexGive(odometry->mutex);
}
