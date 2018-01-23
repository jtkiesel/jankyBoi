#include "Odometry.hpp"

#include "api.hpp"
#include "EncoderWheel.hpp"
#include "Pose.hpp"

#include <cmath>

namespace bns {

Odometry::Odometry(EncoderWheel* const encoderWheelL, EncoderWheel* const encoderWheelR,
		double chassisWidth) : mutex(pros::mutexCreate()), encoderWheelL(encoderWheelL),
		encoderWheelR(encoderWheelR), encoderWheelM(NULL), chassisWidth(chassisWidth) {
	setPose(0, 0, 0);
}

Odometry::Odometry(EncoderWheel* const encoderWheelL, EncoderWheel* const encoderWheelR,
		double chassisWidth, Pose pose) : mutex(pros::mutexCreate()), encoderWheelL(encoderWheelL),
		encoderWheelR(encoderWheelR), encoderWheelM(NULL), chassisWidth(chassisWidth) {
	setPose(pose);
}

Odometry::Odometry(EncoderWheel* const encoderWheelL, EncoderWheel* const encoderWheelR,
		EncoderWheel* const encoderWheelM, double chassisWidth) : mutex(pros::mutexCreate()),
		encoderWheelL(encoderWheelL), encoderWheelR(encoderWheelR), encoderWheelM(encoderWheelM),
		chassisWidth(chassisWidth) {
	setPose(0, 0, 0);
}

Odometry::Odometry(EncoderWheel* const encoderWheelL, EncoderWheel* const encoderWheelR,
		EncoderWheel* const encoderWheelM, double chassisWidth, Pose pose) : mutex(pros::mutexCreate()),
		encoderWheelL(encoderWheelL), encoderWheelR(encoderWheelR), encoderWheelM(encoderWheelM),
		chassisWidth(chassisWidth) {
	setPose(pose);
}

Pose Odometry::computePose() {
	pros::mutexTake(mutex, 20);
	double dL = encoderWheelL->computeDistance() - lastL;
	double dR = encoderWheelR->computeDistance() - lastR;
	double dM = encoderWheelM->computeDistance() - lastM;

	lastL += dL;
	lastR += dR;
	lastM += dM;

	double dS = (dR + dL) / 2;
	double dA = (dR - dL) / chassisWidth;
	double dA_2 = dA / 2;

	double dX = dS * std::sin(dA_2) + dM * std::cos(dA_2);
	double dY = dS * std::cos(dA_2) + dM * std::sin(dA_2);

	pose.add(dX, dY, dA);

	Pose p = pose;
	pros::mutexGive(mutex);
	return p;
}

Pose Odometry::getPose() const {
	pros::mutexTake(mutex, 20);
	Pose p = pose;
	pros::mutexGive(mutex);
	return p;
}

void Odometry::setPose(Pose pose) {
	setPose(pose.x(), pose.x(), pose.theta());
}

void Odometry::setPose(double x, double y, double theta) {
	pros::mutexTake(mutex, 20);
	pose.setX(x);
	pose.setY(y);
	pose.setTheta(theta);
	lastL = encoderWheelL->computeDistance();
	lastR = encoderWheelR->computeDistance();
	lastM = encoderWheelM->computeDistance();
	pros::mutexGive(mutex);
}

}  // namespace bns
