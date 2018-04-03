#include "EncoderWheel.h"

#include "API.h"
#include "util.h"

#include <math.h>

EncoderWheel encoderWheelCreate(Encoder encoder, double countsPerRev, double wheelDiameter,
		double gearRatio, double slipFactor) {
	if (!encoder) {
		printf("Error - encoderWheelCreate: encoder NULL.\n");
		return (EncoderWheel) {};
	}
	return (EncoderWheel) {.encoder = encoder, .countsPerRev = countsPerRev,
			.wheelDiameter = wheelDiameter, .gearRatio = gearRatio, .slipFactor = slipFactor};
}

double encoderWheelDistance(const EncoderWheel* encoderWheel) {
	if (!encoderWheel) {
		printf("Error - encoderWheelDistance: encoderWheel NULL.\n");
		return NAN;
	}
	return (encoderGet(encoderWheel->encoder) * kPi * encoderWheel->wheelDiameter) /
			(encoderWheel->countsPerRev * encoderWheel->gearRatio * encoderWheel->slipFactor);
}
