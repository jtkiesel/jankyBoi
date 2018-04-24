#ifndef GLOBALS_H_
#define GLOBALS_H_

#include "API.h"
#include "Drive.h"
#include "EncoderAnalog.h"
#include "EncoderWheel.h"
#include "Motor.h"
#include "Navigator.h"
#include "Odometry.h"
#include "LineSensor.h"
#include "xsens.h"

// Skills bot mogo tipper ports
#define mogo_tipper_port 23
#define mogo_release_tipper_port 24

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
Encoder encoderLift;

EncoderAnalog encoderRoller;

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

LineSensor leftBarDetect;
LineSensor rightBarDetect;

LineSensor mogoDetect;

Ultrasonic front_left_sonar;
Ultrasonic front_right_sonar;

PidController liftController;

extern PidController* pidTuneController;

void pidTuneTask(void *p);

void compControlTask();

void odometryTask();

void debugTask();

typedef enum MogoState {
	MogoUp,
	MogoDown,
	MogoHoldUp,
} MogoState;
void mogoHoldUp();
void mogoUp();

void mogoDown();

int getMogoPosition();

void mogoTask();

void waitUntilMogo();

typedef enum LiftState {
	LiftUp,
	LiftDown,
	LiftMid,
	LiftLoads,
	LiftPickupLoads
} LiftState;

void liftUp();

void liftDown();

void liftMid();

void liftLoads();
void liftPickupLoads();

int getLiftPosition();

void liftTask();

int getIntakePosition();

typedef enum IntakeState {
	IntakeIn,
	IntakeOut,
	IntakeFullIn,
	IntakeNone
} IntakeState;

void intakeIn();
void intakeFullIn();
void intakeOut();

void intakeNone();

void intakeTask();

#endif  // GLOBALS_H_
