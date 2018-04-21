#include "EncoderAnalog.h"

#include "API.h"

static EncoderAnalogS encState[8];

EncoderAnalog encoderAnalogCreate(unsigned char port) {
	if (port > 8) {
		return &encState[0];
	}
	pinMode(port + 12, INPUT);
	encState[port - 1].lastState = digitalRead(port + 12);
	return &encState[port - 1];
}

unsigned int encoderAnalogCounts(const EncoderAnalog encoderAnalog) {
	return encoderAnalog->counts;
}

void encoderAnalogTask() {
	for (unsigned char i = 0; i < 8; i++) {
		if (digitalRead(i + 13) != encState[i].lastState) {
			encState[i].counts++;
			encState[i].lastState = !encState[i].lastState;
		}
	}
}
