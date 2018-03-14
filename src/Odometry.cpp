#include "Odometry.hpp"

#include "api.hpp"
#include "EncoderWheel.hpp"
#include "Pose.hpp"

#include <cmath>

namespace bns {

Odometry::Odometry(EncoderWheel encoderWheelL, EncoderWheel encoderWheelR, double chassisWidth) :
		Odometry(encoderWheelL, encoderWheelR, chassisWidth, Pose()) {}

Odometry::Odometry(EncoderWheel encoderWheelL, EncoderWheel encoderWheelR, double chassisWidth,
		Pose pose) : kMutex(pros::mutexCreate()), kEncoderWheelL(encoderWheelL),
		kEncoderWheelR(encoderWheelR), kEncoderWheelM(encoderWheelL), kHasEncoderWheelM(false),
		kChassisWidth(chassisWidth) {
	setPose(pose);
}

Odometry::Odometry(EncoderWheel encoderWheelL, EncoderWheel encoderWheelR, EncoderWheel encoderWheelM,
		double chassisWidth) :
		Odometry(encoderWheelL, encoderWheelR, encoderWheelM, chassisWidth, Pose()) {}

Odometry::Odometry(EncoderWheel encoderWheelL, EncoderWheel encoderWheelR,
		EncoderWheel encoderWheelM, double chassisWidth, Pose pose) : kMutex(pros::mutexCreate()),
		kEncoderWheelL(encoderWheelL), kEncoderWheelR(encoderWheelR), kEncoderWheelM(encoderWheelM),
		kHasEncoderWheelM(true), kChassisWidth(chassisWidth) {
	setPose(pose);
}

Odometry::~Odometry() {
	pros::mutexDelete(kMutex);
}

Pose Odometry::computePose() {
	pros::mutexTake(kMutex, 20);
	double dL = kEncoderWheelL.distance() - mLastL;
	double dR = kEncoderWheelR.distance() - mLastR;
	double dM = kHasEncoderWheelM ? (kEncoderWheelM.distance() - mLastM) : 0;

	mLastL += dL;
	mLastR += dR;
	mLastM += dM;

	double dS = (dR + dL) / 2;
	double dA = (dR - dL) / kChassisWidth;
	double dA_2 = dA / 2;

	double dX = dS * std::sin(mPose.theta() + dA_2) + dM * std::cos(dA_2);
	double dY = dS * std::cos(mPose.theta() + dA_2) + dM * std::sin(dA_2);

	mPose.add(dX, dY, dA);

	Pose p = mPose;
	pros::mutexGive(kMutex);
	return p;
}

Pose Odometry::pose() const {
	pros::mutexTake(kMutex, 20);
	Pose p = mPose;
	pros::mutexGive(kMutex);
	return p;
}

void Odometry::setPose(Pose pose) {
	setPose(pose.x(), pose.x(), pose.theta());
}

void Odometry::setPose(double x, double y, double theta) {
	pros::mutexTake(kMutex, 20);
	mLastL = kEncoderWheelL.distance();
	mLastR = kEncoderWheelR.distance();
	mLastM = kHasEncoderWheelM ? kEncoderWheelM.distance() : 0;
	mPose.setX(x);
	mPose.setY(y);
	mPose.setTheta(theta);
	pros::mutexGive(kMutex);
}

}  // namespace bns
