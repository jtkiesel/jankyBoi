#ifndef DRIVE_H_
#define DRIVE_H_

#include "Motor.h"
#include "Odometry.h"

typedef struct Drive {
	Motor* motorLeft;
	Motor* motorRight;
	Motor* motorLeft2;
	Motor* motorRight2;
} Drive;

Drive driveCreate(Motor* motorLeft, Motor* motorRight, Motor* motorLeft2, Motor* motorRight2);

void driveSetPwm(const Drive* drive, int pwmLeft, int pwmRight);

void driveSetPwmAll(const Drive* drive, int pwm);

void driveSetPwmLeft(const Drive* drive, int pwm);

void driveSetPwmRight(const Drive* drive, int pwm);

void driveSetPower(const Drive* drive, double powerLeft, double powerRight);

void driveSetPowerAll(const Drive* drive, double power);

void driveSetPowerLeft(const Drive* drive, double power);

void driveSetPowerRight(const Drive* drive, double power);

#endif  // DRIVE_H_
