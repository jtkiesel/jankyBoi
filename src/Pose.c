#include "Pose.h"

#include "API.h"
#include "log.h"
#include "util.h"
#include "Vector.h"

#include <math.h>

Pose poseCreate(double x, double y, double theta) {
	return (Pose) {.x = x, .y = y, .theta = theta};
}

void poseAdd(Pose* pose, Pose other) {
	if (!pose) {
		logError("poseAdd", "pose NULL");
		return;
	}
	pose->x += other.x;
	pose->y += other.y;
	pose->theta = boundAngleNegPiToPi(pose->theta + other.theta);
}

Vector poseTranslationToPoint(Pose pose, Pose point) {
	double dx = point.x - pose.x;
	double dy = point.y - pose.y;

	return (Vector) {.size = hypot(dx, dy), .angle = boundAngleNegPiToPi(atan2(dy, dx) - pose.theta)};
}

double poseDistanceToPoint(Pose pose, Pose point) {
	double dx = point.x - pose.x;
	double dy = point.y - pose.y;

	return hypot(dx, dy);
}

double poseAngleToPoint(Pose pose, Pose point) {
	double dx = point.x - pose.x;
	double dy = point.y - pose.y;

	return boundAngleNegPiToPi(atan2(dy, dx) - pose.theta);
}
