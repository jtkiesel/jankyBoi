#ifndef ENCODER_HPP_
#define ENCODER_HPP_

#include "api.hpp"

namespace bns {

class Encoder {
public:
	Encoder(unsigned char portA, unsigned char portB);
	Encoder(unsigned char portA, unsigned char portB, bool inverted);
	~Encoder();
	int counts() const;
private:
	pros::Encoder kEncoder;
};

}  // namespace bns

#endif  // ENCODER_HPP_
