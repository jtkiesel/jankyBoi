#ifndef PIDCONTROLLER_HPP_
#define PIDCONTROLLER_HPP_

#include "FeedbackController.hpp"

namespace bns {

class PidController : public FeedbackController {
public:
	PidController(double Kp, double Ki, double Kd);
	double computeOutput(double error, unsigned long t);
	double output() const;
private:
	const double kP;
	const double kI;
	const double kD;
	double mError;
	unsigned long mT;
	double mIntegral;
	double mOutput;
};

}  // namespace bns

#endif  // PIDCONTROLLER_HPP_
