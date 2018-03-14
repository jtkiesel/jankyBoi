#ifndef FEEDBACKCONTROLLER_HPP_
#define FEEDBACKCONTROLLER_HPP_

namespace bns {

class FeedbackController {
public:
	virtual double computeOutput(double error, unsigned long t) = 0;
	virtual double output() const = 0;
};

}  // namespace bns

#endif  // FEEDBACKCONTROLLER_HPP_
