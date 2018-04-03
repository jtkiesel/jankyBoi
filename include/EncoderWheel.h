#ifndef ENCODERWHEEL_H_
#define ENCODERWHEEL_H_

#include "API.h"

typedef struct EncoderWheel {
	Encoder encoder;
	double countsPerRev;
	double wheelDiameter;
	double gearRatio;
	double slipFactor;
} EncoderWheel;

EncoderWheel encoderWheelCreate(Encoder encoder, double countsPerRev, double wheelDiameter,
		double gearRatio, double slipFactor);

double encoderWheelDistance(const EncoderWheel* encoderWheel);

#endif  // ENCODERWHEEL_H_
