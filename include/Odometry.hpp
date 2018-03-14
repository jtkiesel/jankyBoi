#ifndef ODOMETRY_HPP_
#define ODOMETRY_HPP_

#include "api.hpp"
#include "EncoderWheel.hpp"
#include "Pose.hpp"

namespace bns {

class Odometry {
public:
	Odometry(EncoderWheel encoderWheelL, EncoderWheel encoderWheelR, double chassisWidth);
	Odometry(EncoderWheel encoderWheelL, EncoderWheel encoderWheelR, double chassisWidth, Pose pose);
	Odometry(EncoderWheel encoderWheelL, EncoderWheel encoderWheelR, EncoderWheel encoderWheelM, double chassisWidth);
	Odometry(EncoderWheel encoderWheelL, EncoderWheel encoderWheelR, EncoderWheel encoderWheelM, double chassisWidth, Pose pose);
	~Odometry();
	Pose computePose();
	Pose pose() const;
	void setPose(Pose pose);
	void setPose(double x, double y, double theta);
private:
	const pros::Mutex kMutex;
	const EncoderWheel kEncoderWheelL;
	const EncoderWheel kEncoderWheelR;
	const EncoderWheel kEncoderWheelM;
	const bool kHasEncoderWheelM;
	const double kChassisWidth;
	Pose mPose;
	double mLastL;
	double mLastR;
	double mLastM;
};

}  // namespace bns

#endif  // ODOMETRY_HPP_
