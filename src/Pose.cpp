#include "Pose.hpp"

#include <cmath>

#include "api.hpp"
#include "util.hpp"

namespace bns {

Pose::Pose() : Pose(0.0, 0.0, 0.0) {}

Pose::Pose(double x, double y, double theta) : kMutex(pros::mutexCreate()), mX(x), mY(y),
		mTheta(boundAngleNegPiToPi(theta)) {pros::printf("12\n");}

Pose::~Pose() {
	pros::mutexDelete(kMutex);
}

double Pose::x() const {
	return mX;
}

void Pose::setX(double x) {
	pros::mutexTake(kMutex, 20);
	mX = x;
	pros::mutexGive(kMutex);
}

double Pose::y() const {
	return mY;
}

void Pose::setY(double y) {
	pros::mutexTake(kMutex, 20);
	mY = y;
	pros::mutexGive(kMutex);
}

double Pose::theta() const {
	return mTheta;
}

void Pose::setTheta(double theta) {
	pros::mutexTake(kMutex, 20);
	mTheta = boundAngleNegPiToPi(theta);
	pros::mutexGive(kMutex);
}

void Pose::add(Pose other) {
	add(other.x(), other.y(), other.theta());
}

void Pose::add(double x, double y, double theta) {
	pros::mutexTake(kMutex, 20);
	mX += x;
	mY += y;
	mTheta = boundAngleNegPiToPi(mTheta + theta);
	pros::mutexGive(kMutex);
}

Pose Pose::translationToPoint(Pose point) const {
	double dx = point.x() - x();
	double dy = point.y() - y();
pros::printf("11\n");
	return Pose(dx, dy, std::atan2(dy, dx));
}

}  // namespace bns
