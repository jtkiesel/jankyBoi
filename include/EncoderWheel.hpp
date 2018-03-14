#ifndef ENCODERWHEEL_HPP_
#define ENCODERWHEEL_HPP_

#include "Encoder.hpp"

namespace bns {

class EncoderWheel {
public:
	EncoderWheel(Encoder encoder, double countsPerRev, double wheelDiameter);
	EncoderWheel(Encoder encoder, double countsPerRev, double wheelDiameter, double gearRatio);
	EncoderWheel(Encoder encoder, double countsPerRev, double wheelDiameter, double gearRatio,
			double slipFactor);
	double distance() const;
private:
	const Encoder kEncoder;
	const double kCountsPerRev;
	const double kWheelDiameter;
	const double kGearRatio;
	const double kSlipFactor;
};

}  // namespace bns

#endif  // ENCODERWHEEL_HPP_
