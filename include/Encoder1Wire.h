#ifndef ENCODER1WIRE_H_
#define ENCODER1WIRE_H_

typedef struct Encoder1WireS {
	unsigned char port;
	volatile unsigned int counts;
} Encoder1WireS;

typedef Encoder1WireS* Encoder1Wire;

Encoder1Wire encoder1WireCreate(unsigned char port);

unsigned int encoder1WireCounts(const Encoder1Wire enoder1Wire);

#endif  // ENCODER1WIRE_H_
