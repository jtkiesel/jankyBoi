#include "Drive.h"

#include "API.h"
#include "Motor.h"
#include "PidController.h"

#include <math.h>

Drive driveCreate(Motor* motorLeft, Motor* motorLeft2, Motor* motorRight, Motor* motorRight2) {
	if (motorLeft && motorRight) {
		return (Drive) {.motorLeft = motorLeft, .motorLeft2 = motorLeft2, .motorRight = motorRight,
				.motorRight2 = motorRight2};
	}
	printf("Error - driveCreate: motorLeft or motorRight NULL.\n");
	return (Drive) {};
}

void driveSetPwm(const Drive* drive, int pwmLeft, int pwmRight) {
	driveSetPwmLeft(drive, pwmLeft);
	driveSetPwmRight(drive, pwmRight);
}

void driveSetPwmAll(const Drive* drive, int pwm) {
	driveSetPwm(drive, pwm, pwm);
}

void driveSetPwmLeft(const Drive* drive, int pwm) {
	motorSetPwm(drive->motorLeft, pwm);
	motorSetPwm(drive->motorLeft2, pwm);
}

void driveSetPwmRight(const Drive* drive, int pwm) {
	motorSetPwm(drive->motorRight, pwm);
	motorSetPwm(drive->motorRight2, pwm);
}

void driveSetPower(const Drive* drive, double powerLeft, double powerRight) {
	driveSetPowerLeft(drive, powerLeft);
	driveSetPowerRight(drive, powerRight);
}

void driveSetPowerAll(const Drive* drive, double power) {
	int pwm = powerToPwm(power);
	driveSetPwm(drive, pwm, pwm);
}

void driveSetPowerLeft(const Drive* drive, double power) {
	driveSetPwmLeft(drive, powerToPwm(power));
}

void driveSetPowerRight(const Drive* drive, double power) {
	driveSetPwmRight(drive, powerToPwm(power));
}
