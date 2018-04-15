#include "globals.h"

#include "main.h"

#include <math.h>

const unsigned char imeLift = 0;

void compControlTask() {
	int c = fgetc(stdin);
	if (c == 'a') {
		autonomous();
	}
	printf("%d\n", c);
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
bool mogoDone = true;

void mogoUp() {
	mogoDone = false;
	mogoState = MogoUp;
}

void mogoDown() {
	mogoDone = false;
	mogoState = MogoDown;
}

void mogoTask() {
	static MogoState lastMogoState = MogoUp;
	if (mogoState != lastMogoState) {
		if (mogoState == MogoUp) {
			motorSetPower(&motorMogo, 1.0);
			delay(1200);
			motorSetPower(&motorMogo, 0.05);
		} else {
			motorSetPower(&motorMogo, -1.0);
			delay(1200);
			motorSetPower(&motorMogo, -0.05);
		}
		lastMogoState = mogoState;
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

void liftMid() {
	liftState = LiftMid;
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

	if (liftState == LiftDown) {
		error = 0 - liftPosition;
		if (abs(error) < 20) hold = 0.05;
	} else if (liftState == LiftMid){
		error = -275 - liftPosition;
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
	//int value;
	return encoderGet(encoderRoller);
	//return value;
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

void intakeNone()
{
	intakeState = IntakeNone;
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
		if (intakeVelocity > -2) velocity_zero_counter++;
		else velocity_zero_counter = 0;

		if (velocity_zero_counter > 3) motorSetPower(&motorRollers, 0.1);
		else motorSetPower(&motorRollers, 1.0);
	}
	else if (intakeState == IntakeOut)
	{
		motorSetPower(&motorRollers, -1.0);
	}

	printf("Intake Velocity = %d\n", intakeVelocity);

	lastPosition = intakePosition;
}
