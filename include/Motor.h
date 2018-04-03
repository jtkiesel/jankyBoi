#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdbool.h>

typedef struct Motor {
	unsigned char port;
	signed char direction;
} Motor;

Motor motorCreate(unsigned char port, bool reversed);

void motorSetPwm(const Motor* motor, int pwm);

void motorSetPower(const Motor* motor, double power);

/**
 * Converts from output power to PWM value, in order to linearize motor output.
 *
 * @param power  Desired percentage of max power, between -1 and 1.
 * @return       PWM value that will come closest to achieving the desired power.
 */
int powerToPwm(double power);

#endif  // MOTOR_H_
