#include "EncoderWheel.hpp"

#include "api.hpp"
#include "util.hpp"

namespace bns {

EncoderWheel::EncoderWheel(pros::Encoder encoder, double countsPerRev,
		double wheelDiameter, double gearRatio, double slipFactor,
		bool inverted) : kEncoder(encoder), kCountsPerRev(countsPerRev),
		kWheelDiameter(wheelDiameter), kGearRatio(gearRatio),
		kSlipFactor(slipFactor), kInverted(inverted), mDistance(computeDistance()) {
}

double EncoderWheel::computeDistance() {
	mDistance = (counts() * kPi * kWheelDiameter) / (kCountsPerRev * kGearRatio * kSlipFactor);
	return mDistance;
}

double EncoderWheel::distance() const {
	return mDistance;
}

long EncoderWheel::counts() const {
	return (kInverted ? -1 : 1) * pros::encoderGet(kEncoder);
}

}  // namespace bns
