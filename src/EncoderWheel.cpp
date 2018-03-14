#include "EncoderWheel.hpp"

#include "Encoder.hpp"
#include "util.hpp"

namespace bns {

EncoderWheel::EncoderWheel(Encoder encoder, double countsPerRev, double wheelDiameter) :
		EncoderWheel(encoder, countsPerRev, wheelDiameter, 1) {}

EncoderWheel::EncoderWheel(Encoder encoder, double countsPerRev, double wheelDiameter,
		double gearRatio) : EncoderWheel(encoder, countsPerRev, wheelDiameter, gearRatio, 1) {}

EncoderWheel::EncoderWheel(Encoder encoder, double countsPerRev, double wheelDiameter,
		double gearRatio, double slipFactor) : kEncoder(encoder), kCountsPerRev(countsPerRev),
		kWheelDiameter(wheelDiameter), kGearRatio(gearRatio), kSlipFactor(slipFactor) {
}

double EncoderWheel::distance() const {
	return (kEncoder.counts() * kPi * kWheelDiameter) / (kCountsPerRev * kGearRatio * kSlipFactor);
}

}  // namespace bns
