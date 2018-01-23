#ifndef CONSTANTS_HPP_
#define CONSTANTS_HPP_

namespace bns {

/**
 * A list of constants used by the rest of the robot code. This include physics
 * constants as well as constants determined through calibrations.
 */
class Constants {
public:
	/* ROBOT PHYSICAL CONSTANTS */
	// Wheels
	static constexpr double kDriveWheelDiameterInches = 3.25;
	static constexpr double kTrackWidthInches = 8.0;
	static constexpr double kTrackScrubFactor = 1.0;
	// Path following constants
	static constexpr double kMinLookahead = 3.0;  // Inches.
	static constexpr double kMinLookaheadSpeed = 0.9;  // Inches per second.
	static constexpr double kMaxLookahead = 6.0;  // Inches.
	static constexpr double kMaxLookaheadSpeed = 12.0;  // Inches per second.
	static constexpr double kDeltaLookahead = kMaxLookahead - kMinLookahead;
	static constexpr double kDeltaLookaheadSpeed = kMaxLookaheadSpeed - kMinLookaheadSpeed;
	static constexpr double kInertiaSteeringGain = 0.0;
	static constexpr double kSegmentCompletionTolerance = 0.1;  // Inches.
	static constexpr double kPathFollowingMaxAccel = 12.0;  // Inches per second^2.
	static constexpr double kPathFollowingMaxVel = 12.0;  // Inches per second.
	static constexpr double kPathFollowingProfileKp = 5.0;
	static constexpr double kPathFollowingProfileKi = 0.03;
	static constexpr double kPathFollowingProfileKv = 0.02;
	static constexpr double kPathFollowingProfileKffv = 1.0;
	static constexpr double kPathFollowingProfileKffa = 0.05;
	static constexpr double kPathFollowingGoalPosTolerance = 0.75;  // Inches.
	static constexpr double kPathFollowingGoalVelTolerance = 1.2;  // Inches per second.
	static constexpr double kPathStopSteeringDistance = 0.9;  // Inches.
};

// Motors.
const unsigned char kDriveLeft   = 2;
const unsigned char kDriveLeft2  = 3;
const unsigned char kLiftLeft    = 4;
const unsigned char kMogo        = 5;
const unsigned char kLiftRight   = 6;
const unsigned char kDriveRight2 = 7;
const unsigned char kDriveRight  = 8;

// Analog sensors.
const unsigned char kLiftPot = 1;
const unsigned char kMogoPot = 2;
const unsigned char kFourBarSolenoid = 3;
const unsigned char kClawSolenoid = 4;

// Digital sensors.
const unsigned char kDriveQuadLeft    = 1;
const unsigned char kDriveQuadLeft2   = 2;
const unsigned char kDriveQuadRight   = 3;
const unsigned char kDriveQuadRight2  = 4;
const unsigned char kDriveQuadMiddle  = 5;
const unsigned char kDriveQuadMiddle2 = 6;
const unsigned char kConeSonar        = 7;
const unsigned char kConeSonar2       = 8;

// I2C sensors.
const unsigned char kDriveImeLeft  = 0;
const unsigned char kDriveImeRight = 1;

}  // namespace bns

#endif  // CONSTANTS_HPP_
