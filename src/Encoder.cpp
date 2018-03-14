#include "Encoder.hpp"

#include "api.hpp"

namespace bns {

Encoder::Encoder(unsigned char portA, unsigned char portB) : Encoder(portA, portB, false) {}

Encoder::Encoder(unsigned char portA, unsigned char portB, bool inverted) :
		kEncoder(pros::encoderInit(portA, portB, inverted)) {}

Encoder::~Encoder() {
	pros::encoderShutdown(kEncoder);
}

int Encoder::counts() const {
	return pros::encoderGet(kEncoder);
}

}  // namespace bns
