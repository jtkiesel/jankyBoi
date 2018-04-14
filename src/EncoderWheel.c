#include "EncoderWheel.h"

#include "API.h"
#include "log.h"
#include "util.h"

#include <math.h>

EncoderWheel encoderWheelCreate(Encoder encoder, double countsPerRev, double wheelDiameter,
		double gearRatio, double slipFactor) {
	if (!encoder) {
		logError("encoderWheelCreate", "encoder NULL");
		return (EncoderWheel) {};
	}
	return (EncoderWheel) {.encoder = encoder, .countsPerRev = countsPerRev,
			.wheelDiameter = wheelDiameter, .gearRatio = gearRatio, .slipFactor = slipFactor};
}

double encoderWheelDistance(const EncoderWheel* encoderWheel) {
	if (!encoderWheel) {
		logError("encoderWheelDistance", "encoderWheel NULL");
		return NAN;
	}
	return (encoderGet(encoderWheel->encoder) * kPi * encoderWheel->wheelDiameter) /
			(encoderWheel->countsPerRev * encoderWheel->gearRatio * encoderWheel->slipFactor);
}
