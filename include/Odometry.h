#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include "API.h"
#include "EncoderWheel.h"
#include "Pose.h"

typedef struct Odometry {
	Mutex mutex;
	EncoderWheel* encoderWheelL;
	EncoderWheel* encoderWheelR;
	EncoderWheel* encoderWheelM;
	double chassisWidth;
	Pose pose;
	double lastL;
	double lastR;
	double lastM;
} Odometry;

Odometry odometryCreate(EncoderWheel* encoderWheelL, EncoderWheel* encoderWheelR,
		EncoderWheel* encoderWheelM, double chassisWidth, Pose initialPose);

void odometryDelete(Odometry* odometry);

Pose odometryComputePose(Odometry* odometry);

Pose odometryPose(const Odometry* odometry);

void odometrySetPose(Odometry* odometry, Pose pose);

#endif  // ODOMETRY_H_
