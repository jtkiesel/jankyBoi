#ifndef ENCODERANALOG_H_
#define ENCODERANALOG_H_

#include <stdbool.h>

typedef struct EncoderAnalogS {
	unsigned char port;
	volatile unsigned int counts;
	bool lastState;
} EncoderAnalogS;

typedef EncoderAnalogS* EncoderAnalog;

EncoderAnalog encoderAnalogCreate(unsigned char port);

unsigned int encoderAnalogCounts(const EncoderAnalog enoderAnalog);

void encoderAnalogTask();

#endif  // ENCODERANALOG_H_
