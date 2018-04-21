#include "Encoder1Wire.h"

#include "API.h"

static Encoder1WireS encState[12];

static void interruptHandler(unsigned char pin) {
	encState[pin - 1].counts++;
}

Encoder1Wire encoder1WireCreate(unsigned char port) {
	if (port > 12) {
		return &encState[0];
	}
	pinMode(port, INPUT);
	ioSetInterrupt(port, INTERRUPT_EDGE_FALLING, interruptHandler);
	return &encState[port - 1];
}

unsigned int encoder1WireCounts(const Encoder1Wire encoder1Wire) {
	return encoder1Wire->counts;
}
