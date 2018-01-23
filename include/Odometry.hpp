#ifndef ODOMETRY_HPP_
#define ODOMETRY_HPP_

#include "api.hpp"
#include "EncoderWheel.hpp"
#include "Pose.hpp"

namespace bns {

class Odometry {
public:
	Odometry(EncoderWheel* const encoderWheelL, EncoderWheel* const encoderWheelR, double chassisWidth);
	Odometry(EncoderWheel* const encoderWheelL, EncoderWheel* const encoderWheelR, double chassisWidth, Pose initialPose);
	Odometry(EncoderWheel* const encoderWheelL, EncoderWheel* const encoderWheelR, EncoderWheel* const encoderWheelM, double chassisWidth);
	Odometry(EncoderWheel* const encoderWheelL, EncoderWheel* const encoderWheelR, EncoderWheel* const encoderWheelM, double chassisWidth, Pose initialPose);
	Pose computePose();
	Pose getPose() const;
	void setPose(Pose pose);
	void setPose(double x, double y, double theta);
private:
	const pros::Mutex mutex;
	EncoderWheel* const encoderWheelL;
	EncoderWheel* const encoderWheelR;
	EncoderWheel* const encoderWheelM;
	const double chassisWidth;
	Pose pose;
	double lastL;
	double lastR;
	double lastM;
};

}  // namespace bns

#endif  // ODOMETRY_HPP_
