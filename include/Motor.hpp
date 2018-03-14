#ifndef MOTOR_HPP_
#define MOTOR_HPP_

namespace bns {

class Motor {
public:
	Motor(unsigned char port, bool reversed = false);
	void setPwm(int pwm) const;
	void setPower(double power) const;
private:
	const unsigned char port;
	const char direction;
	/**
	 * Converts from output power to PWM value, in order to linearize motor output.
	 *
	 * @param power  Desired percentage of max power, between -1 and 1.
	 * @return       PWM value that will come closest to achieving the desired power.
	 */
	static int powerToPwm(double power);
};

}  // namespace bns

#endif  // SUBSYSTEM_HPP_
