#ifndef POSE_H_
#define POSE_H_

#include "Vector.h"

typedef struct Pose {
	double x;
	double y;
	double theta;
} Pose;

Pose poseCreate(double x, double y, double theta);

void poseAdd(Pose* pose, Pose other);

Vector poseTranslationToPoint(Pose pose, Pose point);

double poseDistanceToPoint(Pose pose, Pose point);

double poseAngleToPoint(Pose pose, Pose point);

#endif  // POSE_HPP_
