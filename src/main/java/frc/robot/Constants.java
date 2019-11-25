/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import lib.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class JoystickConstants {
        public static final int kJoystickPort = 0;
        public static final double kJoystickThreshold = 0.2;
    }

    public static final class DrivetrainConstants {
        public static final int kmLeftFront = 1;
        public static final int kmLeftBack = 2;
        public static final int kmRightFront = 3;
        public static final int kmRightBack = 4;
    }

    
    public static final class MotionConstants {
        public static double kEpsilon = 1e-6;
    }


    public static final double kLooperDt = 0.01;

    // wheels
    // Tuned 3/26/19
    public static final double kDriveWheelTrackWidthInches = 25.42;
    public static final double kDriveWheelDiameterInches = 3.938;
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kDriveWheelTrackRadiusWidthMeters = kDriveWheelTrackWidthInches / 2.0 * 0.0254;
    public static final double kTrackScrubFactor = 1.0469745223;

    // tuned dynamics
    public static final double kDriveLinearVIntercept = 0.1801; // V
    public static final double kDriveLinearKv = 0.0919; // V per rad/s
    public static final double kDriveLinearKa = 0.03344; // V per rad/s^2
    public static final double kDriveAngularKa = 0.02897 * 2.0; // V per rad/s^2
    public static final double kRobotLinearInertia = 63.9565; // kg
    public static final double kRobotAngularInertia = kDriveAngularKa / kDriveLinearKa *
            kDriveWheelTrackRadiusWidthMeters * kDriveWheelTrackRadiusWidthMeters * kRobotLinearInertia;  // kg m^2
    public static final double kRobotAngularDrag = 0.0; // N*m / (rad/sec)

    public static final int kCANTimeoutMs = 10; // use for important on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors

    public static final double kBumperHeight = 6.6 + 2.0; // inches to ground + 2 in buffer

    // Do not change anything after this line unless you rewire the robot and
    // update the spreadsheet!
    // Port assignments should match up with the spreadsheet here:
    // https://docs.google.com/spreadsheets/d/1w9V3_tqQ0npdc9U8WPD-6zJkKouunKzHvPXLbHEWwxk/edit#gid=0

    // drive
    public static final int kLeftDriveMasterId = 0;
    public static final int kLeftDriveSlaveId = 1;
    public static final int kRightDriveMasterId = 2;
    public static final int kRightDriveSlaveId = 3;

    public static final int kLeftDriveEncoderA = 0;
    public static final int kLeftDriveEncoderB = 1;
    public static final int kRightDriveEncoderA = 2;
    public static final int kRightDriveEncoderB = 3;

    public static final double kDriveEncoderPPR = 1000.0;

    public static final double kMinLookAhead = 12.0; // inches
    public static final double kMinLookAheadSpeed = 12.0; // inches per second
    public static final double kMaxLookAhead = 48.0; // inches
    public static final double kMaxLookAheadSpeed = 120.0; // inches per second
    public static final double kDeltaLookAhead = kMaxLookAhead - kMinLookAhead;
    public static final double kDeltaLookAheadSpeed = kMaxLookAheadSpeed - kMinLookAheadSpeed;

    public static final double kInertiaSteeringGain = 0.0; // angular velocity command is multiplied by this gain *
                                                     // our speed
                                                     // in inches per sec
    public static final double kPathFollowingMaxAccel = 80.0;  // inches per second ^ 2
    public static final double kPathFollowingMaxVel = 120.0; // inches per second
    public static final double kPathFollowingProfileKp = 0.3 / 12.0;  // % throttle per inch of error
    public static final double kPathFollowingProfileKi = 0.0;
    public static final double kPathFollowingProfileKv = 0.01 / 12.0;  // % throttle per inch/s of error
    public static final double kPathFollowingProfileKffv = 0.003889;  // % throttle per inch/s
    public static final double kPathFollowingProfileKffa = 0.001415;  // % throttle per inch/s^2
    public static final double kPathFollowingProfileKs = 0.1801 / 12.0;  // % throttle
    public static final double kPathFollowingGoalPosTolerance = 3.0;
    public static final double kPathFollowingGoalVelTolerance = 12.0;
    public static final double kPathStopSteeringDistance = 12.0;
    public static final double kDriveVoltageRampRate = 0.0;
    public static final int kDriveCurrentThrottledLimit = 30; // amps
    public static final int kDriveCurrentUnThrottledLimit = 80; // amps

    public static final double kFollowTargetSamplingDistance = 1.0;
    public static final double kFollowTargetLookahead = 30.0;
    public static final double kFollowTargetTolerance = 0.1;
    public static final double kFollowTargetSteeringScale = 0.05 * 24;
    public static final double kFollowTargetMaxThrottle = 0.8;
    public static final double kFollowTargetExtraWaypointDistance = 30.0;
    public static final double kPathKX = 4.0; // units/s per unit of error
    public static final double kPathLookaheadTime = 0.4; // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0; // inches

    // reset button
    public static final int kResetButtonChannel = 4;

    // canifier
    public static final int kCanifierArmId = 16;
    public static final int kCanifierWristId = 9;

    // pigeon imu
    public static final int kPigeonIMUId = 15;

    // Navx
    public static final int kNavXId = 1;


    public static final boolean kUseDriveGamepad = false;
    public static final int kDriveGamepadPort = 0;
    public static final int kButtonGamepadPort = 0;
    public static final int kMainThrottleJoystickPort = 0;
    public static final int kMainTurnJoystickPort = 1;
    public static final double kJoystickThreshold = 0.2;

    // solenoids
    public static final int kPCMId = 1;
    public static final int kShifterSolenoidId = 7;
    public static final int kBallIntakeJawId = 6;
    public static final int kKickstandForwardId = 1; // deploys
    public static final int kKickstandReverseId = 0; // retracts
    public static final int kRatchetForwardId = 3; // deploys
    public static final int kRatchetReverseId = 2; // retracts

    // limelight
    public static final double kHorizontalFOV = 59.6; // degrees
    public static final double kVerticalFOV = 49.7; // degrees
    public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
    public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));
    public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds

    public static final double kMaxTrackerDistance = 9.0;
    public static final double kMaxGoalTrackAge = 2.5;
    public static final double kMaxGoalTrackAgeNotTracking = 0.1;
    public static final double kMaxGoalTrackSmoothingTime = 0.5;
    public static final double kTrackStabilityWeight = 0.0;
    public static final double kTrackAgeWeight = 10.0;
    public static final double kTrackSwitchingWeight = 100.0;

    public static final double kCameraFrameRate = 90.0;
    public static final double kMinStability = 0.5;
    public static final int kPortPipeline = 0;
    public static final int kBallPipeline = 2;
    public static final double kPortTargetHeight = 39.125;
    public static final double kHatchTargetHeight = 31.5;

    public static final double kTurretToArmOffset = -2.5;  // in
    public static final double kWristToTremorsEnd = 15.75;  // in

    public static final double kMaxTopLimelightHeight = 16.0;

    public static final double kGenerateTrajectoryTime = 0.5;
    public static final double kUseNextTrajectoryTime = 0.75;
    public static final Rotation2d kMaxDeviance = Rotation2d.fromDegrees(0); // max angle away from ball that robot can be and still pick it up

    // Drive control
    public static final double kStingerForwardPower = 0.8;
    public static final double kClimbingElevatorHeightForLowShift = 10.0; // in


}
