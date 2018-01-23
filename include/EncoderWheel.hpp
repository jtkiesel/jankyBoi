#ifndef ENCODERWHEEL_HPP_
#define ENCODERWHEEL_HPP_

#include "api.hpp"

namespace bns {

class EncoderWheel {
public:
	EncoderWheel(pros::Encoder encoder, double countsPerRev, double wheelDiameter,
			double gearRatio, double slipFactor, bool inverted);
	double computeDistance();
	double distance() const;
private:
	const pros::Encoder kEncoder;
	const double kCountsPerRev;
	const double kWheelDiameter;
	const double kGearRatio;
	const double kSlipFactor;
	const bool kInverted;
	double mDistance;
	long counts() const;
};

}  // namespace bns

#endif  // ENCODERWHEEL_HPP_
