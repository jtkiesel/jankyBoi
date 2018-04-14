#include "globals.h"

#include "main.h"

#include <math.h>

const unsigned char imeRollers = 2;
const unsigned char imeLift = 1;
const unsigned char imeMogo = 0;

void compControlTask() {
	if (fgetc(stdin) == 'a') {
		autonomous();
	}
}

void odometryTask() {
	odometryComputePose(&odometry);
}

void debugTask() {
	printf("(%.3f x, %.3f y, %f theta)\n", odometry.pose.x, odometry.pose.y, odometry.pose.theta);
	//printf("xsens yaw: %.3f\n", xsens_get_yaw(&xsens));
	//const Vector translation = poseTranslationToPoint(odometry.pose, (Pose) {.x = 30.0, .y = 0.0, .theta = 0.0});//odometryPose(&odometry);
	//printf("translation: (%.3f size, %f angle)\n", translation.size, translation.angle);
}

MogoState mogoState = MogoUp;

void mogoUp() {
	mogoState = MogoUp;
}

void mogoDown() {
	mogoState = MogoDown;
}

int getMogoPosition() {
	int value;
	imeGet(imeMogo, &value);
	return value;
}

void mogoTask() {
	if (mogoState == MogoUp) {
		if (getMogoPosition() < 1000) {
			motorSetPower(&motorMogo, 1.0);
		} else {
			motorSetPower(&motorMogo, 0.05);
		}
	} else {
		while (getMogoPosition() > 100) {
			motorSetPower(&motorMogo, -1.0);
			delay(20);
		}
		motorSetPower(&motorMogo, -0.05);
	}
}

void waitUntilMogo() {
	if (mogoState == MogoUp) {
		while (getMogoPosition() < (0 - 60)) {
			delay(20);
		}
	} else {
		while (getMogoPosition() > (-625 + 60)) {
			delay(20);
		}
	}
}

LiftState liftState = LiftUp;

void liftUp() {
	liftState = LiftUp;
}

void liftDown() {
	liftState = LiftDown;
}

int getLiftPosition() {
	int value;
	imeGet(imeLift, &value);
	return value;
}

void liftTask() {

	int liftPosition = getLiftPosition();
	double error = 0;
	double hold = 0;

	if (liftState == LiftUp) {
		error = 0 - liftPosition;
		if (abs(error) < 20) hold = 0.05;
	} else if (liftState == LiftMid){
		error = -175 - liftPosition;
		//if (abs(error) < 20) hold = 0.05;
	}
	else {
		error = -950 - liftPosition;
		if (abs(error) < 20) hold = -0.05;
	}

	double pid_output = pidControllerComputeOutput(&liftController, error, 1);

	if (hold != 0)
		motorSetPower(&motorLift, hold);
	else
	  motorSetPower(&motorLift, pid_output);
	//printf("lift position %d, output = %f\n", liftPosition, pid_output);
}

int getIntakePosition()
{
	int value;
	imeGet(imeRollers, &value);
	return value;
}

IntakeState intakeState = IntakeNone;

void intakeIn()
{
	intakeState = IntakeIn;
}

void intakeOut()
{
	intakeState = IntakeOut;
}

void intakeTask()
{
	static int lastPosition = 0;
	static int velocity_zero_counter = 0;

	int intakePosition = getIntakePosition();
	int intakeVelocity = intakePosition - lastPosition;

	if (intakeState == IntakeNone)
	{
		motorSetPower(&motorRollers, 0);
	}
	else if (intakeState == IntakeIn)
	{
		if (intakeVelocity < 2) velocity_zero_counter++;
		else velocity_zero_counter = 0;

		if (velocity_zero_counter > 3) motorSetPower(&motorRollers, 0.05);
		else motorSetPower(&motorRollers, 1.0);
	}
	else if (intakeState == IntakeOut)
	{
		motorSetPower(&motorRollers, -1.0);
	}

	//printf("Intake Velocity = %d\n", intakeVelocity);

	lastPosition = intakePosition;
}
