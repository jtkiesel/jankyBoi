#include "Drive.h"

#include "log.h"
#include "Motor.h"
#include "PidController.h"

#include <math.h>

Drive driveCreate(Motor* motorLeft, Motor* motorRight, Motor* motorLeft2, Motor* motorRight2) {
	if (!motorLeft) {
		logError("driveCreate", "motorLeft NULL");
		return (Drive) {};
	}
	if (!motorRight) {
		logError("driveCreate", "motorRight NULL");
		return (Drive) {};
	}
	return (Drive) {.motorLeft = motorLeft, .motorRight = motorRight, .motorLeft2 = motorLeft2,
			.motorRight2 = motorRight2};
}

void driveSetPwm(const Drive* drive, int pwmLeft, int pwmRight) {
	if (!drive) {
		logError("driveSetPwm", "drive NULL");
		return;
	}
	driveSetPwmLeft(drive, pwmLeft);
	driveSetPwmRight(drive, pwmRight);
}

void driveSetPwmAll(const Drive* drive, int pwm) {
	if (!drive) {
		logError("driveSetPwmAll", "drive NULL");
		return;
	}
	driveSetPwm(drive, pwm, pwm);
}

void driveSetPwmLeft(const Drive* drive, int pwm) {
	if (!drive) {
		logError("driveSetPwmLeft", "drive NULL");
		return;
	}
	motorSetPwm(drive->motorLeft, pwm);
	if (drive->motorLeft2) {
		motorSetPwm(drive->motorLeft2, pwm);
	}
}

void driveSetPwmRight(const Drive* drive, int pwm) {
	if (!drive) {
		logError("driveSetPwmRight", "drive NULL");
		return;
	}
	motorSetPwm(drive->motorRight, pwm);
	if (drive->motorRight2) {
		motorSetPwm(drive->motorRight2, pwm);
	}
}

void driveSetPower(const Drive* drive, double powerLeft, double powerRight) {
	if (!drive) {
		logError("driveSetPower", "drive NULL");
		return;
	}
	driveSetPowerLeft(drive, powerLeft);
	driveSetPowerRight(drive, powerRight);
}

void driveSetPowerAll(const Drive* drive, double power) {
	if (!drive) {
		logError("driveSetPowerAll", "drive NULL");
		return;
	}
	int pwm = powerToPwm(power);
	driveSetPwm(drive, pwm, pwm);
}

void driveSetPowerLeft(const Drive* drive, double power) {
	if (!drive) {
		logError("driveSetPowerLeft", "drive NULL");
		return;
	}
	driveSetPwmLeft(drive, powerToPwm(power));
}

void driveSetPowerRight(const Drive* drive, double power) {
	if (!drive) {
		logError("driveSetPowerRight", "drive NULL");
		return;
	}
	driveSetPwmRight(drive, powerToPwm(power));
}
