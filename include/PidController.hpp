#ifndef PIDCONTROLLER_HPP_
#define PIDCONTROLLER_HPP_

#include "FeedbackController.hpp"

namespace bns {

class PidController : public FeedbackController {
public:
	PidController(double Kp, double Ki, double Kd);
	double computeOutput(double error, unsigned long t);
	double getOutput() const;
private:
	const double Kp;
	const double Ki;
	const double Kd;
	double error;
	unsigned long t;
	double integral;
	double output;
};

}  // namespace bns

#endif  // PIDCONTROLLER_HPP_
