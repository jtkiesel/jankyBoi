#include "globals.h"

#include "asciitof.h"
#include "Motor.h"
#include "Encoder1Wire.h"
#include "main.h"
#include "Odometry.h"

#include <math.h>

const unsigned char imeLift = 0;

void compControlTask() {
	int c = fgetc(stdin);
	if (c == 'a') {
		autonomous();
	}
}

void odometryTask() {
	odometryComputePose(&odometry);
}

void debugTask() {
	//printf("(%.3f x, %.3f y, %f theta)\n", odometry.pose.x, odometry.pose.y, odometry.pose.theta);
	//printf("xsens yaw: %.3f\n", xsens_get_yaw(&xsens));
	//const Vector translation = poseTranslationToPoint(odometry.pose, (Pose) {.x = 30.0, .y = 0.0, .theta = 0.0});//odometryPose(&odometry);
	//printf("translation: (%.3f size, %f angle)\n", translation.size, translation.angle);
	//printf("analog: %d\n", analogRead(3));
	//printf("ultrasonic right: %d\n", ultrasonicGet(front_right_sonar));
	//printf("encoderRoller: %d\n", encoderAnalogCounts(encoderRoller));
}

PidController* pidTuneController = &navigator.turnController;

void pidTuneTask(void *p) {
	while (true) {
		int c = fgetc(stdin);
		if (c == 'p') {
			char s[20];
			int i = 0;
			while (c != '\n' && i < 20) {
				c = fgetc(stdin);
				s[i++] = c;
			}
			pidTuneController->Kp = asciitof(s);
		} else if (c == 'd') {
			char s[20];
			int i = 0;
			while (c != '\n' && i < 20) {
				c = fgetc(stdin);
				s[i++] = c;
			}
			pidTuneController->Kd = asciitof(s);
		}
		printf("Kp: %f Kd: %f\n", pidTuneController->Kp, pidTuneController->Kd);
		delay(100);
	}
}

MogoState mogoState = MogoUp;
bool mogoDone = true;

void mogoUp() {
	mogoDone = false;
	mogoState = MogoUp;
}

void mogoDown() {
	mogoDone = false;
	mogoState = MogoDown;
}

void mogoHoldUp() {
	mogoDone = true;
	mogoState = MogoHoldUp;
}

void mogoTask() {
	static MogoState lastMogoState = MogoUp;
	if (mogoState == MogoHoldUp)
	{
		motorSetPwm(&motorMogo, 30);
	}
	else if (mogoState != lastMogoState) {
		lastMogoState = mogoState;
		if (mogoState == MogoUp) {
			motorSetPower(&motorMogo, 1.0);
			delay(1000);
			motorSetPwm(&motorMogo, 15);
		} else {
			motorSetPower(&motorMogo, -1.0);
			delay(1000);
			motorSetPwm(&motorMogo, -15);
		}
		mogoDone = true;
	}
}

void waitUntilMogo() {
	while (!mogoDone) {
		delay(20);
	}
}

LiftState liftState = LiftDown;

void liftUp() {
	liftState = LiftUp;
}

void liftDown() {
	liftState = LiftDown;
}

void liftLoads() {
	liftState = LiftLoads;
}

void liftPickupLoads() {
	liftState = LiftPickupLoads;
}

void liftMid() {
	liftState = LiftMid;
}

int getLiftPosition() {
	return encoderGet(encoderLift);
}

void liftTask() {
	int liftPosition = getLiftPosition();
	double error = 0;
	double hold = 0;

	if (liftState == LiftDown) {
		error = 0 - liftPosition;
		if (fabs(error) < 20) {
			hold = 0.05;
		}
	} else if (liftState == LiftMid){
		error = -275 - liftPosition;
		//if (abs(error) < 20) hold = 0.05;
	} else if (liftState == LiftLoads) {
		error = -600 - liftPosition;
	} else if (liftState == LiftPickupLoads) {
		error = -800 - liftPosition;
	} else {
		error = -1325 - liftPosition;
		if (fabs(error) < 20) {
			hold = -0.1;
		}
	}
	double pidOutput = pidControllerComputeOutput(&liftController, -error, 1);

	if (hold != 0) {
		motorSetPower(&motorLift, hold);
	} else {
		motorSetPower(&motorLift, pidOutput);
	}
	//printf("lift position %d, output = %f\n", liftPosition, pidOutput);
}

int getIntakePosition() {
	return (int) encoderAnalogCounts(encoderRoller);
}

IntakeState intakeState = IntakeNone;

void intakeIn() {
	intakeState = IntakeIn;
}

void intakeOut() {
	intakeState = IntakeOut;
}

void intakeNone() {
	intakeState = IntakeNone;
}

void intakeFullIn() {
	intakeState = IntakeFullIn;
}

void intakeTask() {
	static int lastPosition = 0;
	static int velocityZeroCounter = 0;

	int intakePosition = getIntakePosition();
	int intakeVelocity = intakePosition - lastPosition;

	if (intakeState == IntakeFullIn) {
		motorSetPower(&motorRollers, 1.0);
		//printf("IntakeFullin\n");
	}
	else if (intakeState == IntakeNone) {
		motorSetPower(&motorRollers, 0.15);
		velocityZeroCounter = 0;
		//printf("IntakeNone\n");
	} else if (intakeState == IntakeIn) {
		if (abs(intakeVelocity) < 2) {
			velocityZeroCounter++;
		} else {
			velocityZeroCounter = 0;
		}
		if (velocityZeroCounter > 4) {
			//printf("QuitingIn\n");
			motorSetPower(&motorRollers, 0.15);
			intakeState = IntakeNone;
		} else {
		//	printf("Running in");
			motorSetPower(&motorRollers, 1.0);
		}
	} else if (intakeState == IntakeOut) {
		//printf("Running out");
		velocityZeroCounter = 0;
		motorSetPower(&motorRollers, -1.0);
	} else {
		velocityZeroCounter = 0;
	}
	//printf("IV = %d %d\n", intakeVelocity);

	lastPosition = intakePosition;
}
