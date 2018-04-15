#ifndef GLOBALS_H_
#define GLOBALS_H_

#include "API.h"
#include "Drive.h"
#include "EncoderWheel.h"
#include "Motor.h"
#include "Navigator.h"
#include "Odometry.h"
#include "LineSensor.h"
#include "xsens.h"

Motor motorDriveL;
Motor motorDriveL2;
Motor motorRollers;
Motor motorLift;
Motor motorMogo;
Motor motorDriveR;
Motor motorDriveR2;

Encoder encoderDriveL;
Encoder encoderDriveR;
Encoder encoderDriveM;

Encoder encoderRoller;

extern const unsigned char imeLift;

EncoderWheel encoderWheelL;
EncoderWheel encoderWheelR;
EncoderWheel encoderWheelM;
struct XsensVex xsens;
Odometry odometry;

Drive drive;
Navigator navigator;

LineSensor leftLine;
LineSensor rightLine;
LineSensor backLine;

PidController liftController;

void compControlTask();

void odometryTask();

void debugTask();

typedef enum MogoState {
	MogoUp,
	MogoDown
} MogoState;

void mogoUp();

void mogoDown();

int getMogoPosition();

void mogoTask();

void waitUntilMogo();

typedef enum LiftState {
	LiftUp,
	LiftDown,
	LiftMid
} LiftState;

void liftUp();

void liftDown();

void liftMid();

int getLiftPosition();

void liftTask();

int getIntakePosition();

typedef enum IntakeState {
	IntakeIn,
	IntakeOut,
	IntakeNone
} IntakeState;

void intakeIn();

void intakeOut();

void intakeNone();

void intakeTask();

#endif  // GLOBALS_H_
