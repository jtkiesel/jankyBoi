#include "Pose.hpp"

#include "api.hpp"

namespace bns {

Pose::Pose() : Pose(0.0, 0.0, 0.0) {
}

Pose::Pose(double x, double y, double theta) : mutex(pros::mutexCreate()), mX(x), mY(y), mTheta(theta) {
}

double Pose::x() const {
	return mX;
}

void Pose::setX(double x) {
	pros::mutexTake(mutex, 20);
	mX = x;
	pros::mutexGive(mutex);
}

double Pose::y() const {
	return mY;
}

void Pose::setY(double y) {
	pros::mutexTake(mutex, 20);
	mY = y;
	pros::mutexGive(mutex);
}

double Pose::theta() const {
	return mTheta;
}

void Pose::setTheta(double theta) {
	pros::mutexTake(mutex, 20);
	mTheta = theta;
	pros::mutexGive(mutex);
}

void Pose::add(Pose pose) {
	add(pose.mX, pose.mY, pose.mTheta);
}

void Pose::add(double x, double y, double theta) {
	pros::mutexTake(mutex, 20);
	mX += x;
	mY += y;
	mTheta += theta;
	pros::mutexGive(mutex);
}

}  // namespace bns
