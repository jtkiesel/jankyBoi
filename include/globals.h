#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#include "API.h"
#include "Drive.h"
#include "EncoderWheel.h"
#include "Motor.h"
#include "Navigator.h"
#include "Odometry.h"

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

extern const unsigned char imeRollers;
extern const unsigned char imeLift;
extern const unsigned char imeMogo;

EncoderWheel encoderWheelL;
EncoderWheel encoderWheelR;
EncoderWheel encoderWheelM;
Odometry odometry;

Drive drive;
Navigator navigator;

#endif  // CONSTANTS_H_
