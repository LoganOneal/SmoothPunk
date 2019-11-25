/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.testing;

// Wpilib
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.SPI;

// only use for testing
import edu.wpi.first.wpilibj.Spark;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.ControlType;
import frc.robot.Constants;
import lib.control.Lookahead;
import lib.control.Path;
import lib.control.PathFollower;
import lib.geometry.*;
import lib.util.*;
import lib.vision.AimingParameters;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Kinematics;
import frc.robot.RobotState;

// Motors
import frc.robot.drivers.motors.sparkMAX.LazySparkMax;
import frc.robot.drivers.motors.sparkMAX.SparkMaxFactory;


// Sensors
import frc.robot.drivers.sensors.NavX;
// Maths
import lib.geometry.Rotation2d;

// Paths
import lib.control.Path;
import lib.control.PathFollower;

//Constants


/**
 * Subsystem that holds everything for the drivetrain.
 * 
 * Includes: motors and drivetrain methods.
 * 
 * Robot operates off of a Cartesian coordinate system.
 */
public class Drive extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Spark mLeftMaster, mRightMaster, mLeftSlave, mRightSlave;

  private final Encoder mLeftEncoder, mRightEncoder;

  // Controllers
  private PathFollower mPathFollower;
  private Path mCurrentPath = null;

  private final Solenoid mShifter;

  // control states
  private DriveControlState mDriveControlState;
  private DriveCurrentLimitState mDriveCurrentLimitState;

  //Navigation Sensors
  //private PigeonIMU mPigeon;
  private NavX mNavx;

  // hardware states
  private boolean mIsHighGear;
  private boolean mIsBrakeMode;
  private Rotation2d mGyroOffset = Rotation2d.identity();;
  private double mLastDriveCurrentSwitchTime = -1;

  private void configureSpark(Spark sparkMax, boolean left, boolean master) {
    sparkMax.setInverted(!left);
  }

  public Drive() {
    mPeriodicIO = new PeriodicIO();

    mNavx = new NavX(SPI.Port.kMXP);
     
    mLeftMaster = new Spark(Constants.kLeftDriveMasterId);
    configureSpark(mLeftMaster, true, true);
    System.out.println("MOTORS DECLARED");

    mLeftSlave = new Spark(Constants.kLeftDriveSlaveId);
    configureSpark(mLeftSlave, true, false);

    mRightMaster = new Spark(Constants.kRightDriveMasterId);
    configureSpark(mRightMaster, false, true);

    mRightSlave = new Spark(Constants.kRightDriveSlaveId);
    configureSpark(mRightSlave, false, false);


    mLeftEncoder = new Encoder(Constants.kLeftDriveEncoderA, Constants.kLeftDriveEncoderB, false);
    mRightEncoder = new Encoder(Constants.kRightDriveEncoderA, Constants.kRightDriveEncoderB, true);

    mLeftEncoder.setReverseDirection(true);
    mRightEncoder.setReverseDirection(false);

    mLeftEncoder.setDistancePerPulse(Constants.kDriveWheelDiameterInches * Math.PI / Constants.kDriveEncoderPPR);
    mRightEncoder.setDistancePerPulse(Constants.kDriveWheelDiameterInches * Math.PI / Constants.kDriveEncoderPPR);

    mShifter = new Solenoid(Constants.kPCMId, Constants.kShifterSolenoidId);

    mIsHighGear = false;
    setHighGear(true);

    setOpenLoop(DriveSignal.NEUTRAL);

    // force a CAN message across
    mIsBrakeMode = true;
    setBrakeMode(false);

    mDriveCurrentLimitState = DriveCurrentLimitState.UNTHROTTLED;
    setDriveCurrentState(DriveCurrentLimitState.THROTTLED, 0.0);
  }

  private PeriodicIO mPeriodicIO;
  private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

  public static class PeriodicIO {
      // INPUTS
      public double timestamp;
      public double left_voltage;
      public double right_voltage;
      public int left_position_ticks;
      public int right_position_ticks;
      public double left_distance;
      public double right_distance;
      public int left_velocity_ticks_per_100ms;
      public int right_velocity_ticks_per_100ms;
      public Rotation2d gyro_heading = Rotation2d.identity();
      public Pose2d error = Pose2d.identity();

      // OUTPUTS
      public double left_demand;
      public double right_demand;
      public double left_accel;
      public double right_accel;
      public double left_feedforward;
      public double right_feedforward;
  }

  public synchronized void readPeriodicInputs() {
    mPeriodicIO.timestamp = Timer.getFPGATimestamp();

    double prevLeftTicks = mPeriodicIO.left_position_ticks;
    double prevRightTicks = mPeriodicIO.right_position_ticks;
    //mPeriodicIO.left_voltage = mLeftMaster.getAppliedOutput() * mLeftMaster.getBusVoltage();
    //mPeriodicIO.right_voltage = mRightMaster.getAppliedOutput() * mRightMaster.getBusVoltage();

    mPeriodicIO.left_position_ticks = mLeftEncoder.get();
    mPeriodicIO.right_position_ticks = mRightEncoder.get();

    //mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(mPigeon.getFusedHeading()).rotateBy(mGyroOffset);
    mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(mNavx.getRawYawDegrees()).rotateBy(mGyroOffset);

    double deltaLeftTicks = ((mPeriodicIO.left_position_ticks - prevLeftTicks) / Constants.kDriveEncoderPPR)
            * Math.PI;
    mPeriodicIO.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;

    double deltaRightTicks = ((mPeriodicIO.right_position_ticks - prevRightTicks) / Constants.kDriveEncoderPPR)
            * Math.PI;
    mPeriodicIO.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;

    mPeriodicIO.left_velocity_ticks_per_100ms = (int) (mLeftEncoder.getRate()
            / (10 * mLeftEncoder.getDistancePerPulse()));
    mPeriodicIO.right_velocity_ticks_per_100ms = (int) (mRightEncoder.getRate()
            / (10 * mRightEncoder.getDistancePerPulse()));

    if (mCSVWriter != null) {
        mCSVWriter.add(mPeriodicIO);
    }
  }

  public synchronized void setCheesyishDrive(double throttle, double wheel, boolean quickTurn) {
    if (Util.epsilonEquals(throttle, 0.0, 0.04)) {
        throttle = 0.0;
    }

    if (Util.epsilonEquals(wheel, 0.0, 0.035)) {
        wheel = 0.0;
    }

    final double kWheelGain = 0.05;
    final double kWheelNonlinearity = 0.05;
    final double denominator = Math.sin(Math.PI / 2.0 * kWheelNonlinearity);
    // Apply a sin function that's scaled to make it feel better.
    if (!quickTurn) {
        wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
        wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
        wheel = wheel / (denominator * denominator) * Math.abs(throttle);
    }

    wheel *= kWheelGain;
    DriveSignal signal = Kinematics.inverseKinematics(new Twist2d(throttle, 0.0, wheel));
    double scaling_factor = Math.max(1.0, Math.max(Math.abs(signal.getLeft()), Math.abs(signal.getRight())));
    setOpenLoop(new DriveSignal(signal.getLeft() / scaling_factor, signal.getRight() / scaling_factor));
}

  public synchronized void writePeriodicOutputs() {
    if (mLeftMaster != null) {
      System.out.println("Not null");
      if (mDriveControlState == DriveControlState.OPEN_LOOP) {
          mLeftMaster.set(mPeriodicIO.left_demand);
          mRightMaster.set(mPeriodicIO.right_demand);
      } else {
          mLeftMaster.set(mPeriodicIO.left_demand);
          mRightMaster.set(mPeriodicIO.right_demand);
      }
    }
  }

  public void onLoop(double timestamp) {
    synchronized (Drive.this) {
        handleFaults();
        switch (mDriveControlState) {
            case OPEN_LOOP:
                break;
            case PATH_FOLLOWING:
                if (mPathFollower != null) {
                    updatePathFollower(timestamp);
                }
                break;
            default:
                System.out.println("unexpected drive control state: " + mDriveControlState);
                break;
        }
    }
  }

  public synchronized void setHighGear(boolean wantsHighGear) {
    if (wantsHighGear != mIsHighGear) {
        mIsHighGear = wantsHighGear;
        // Plumbed default high.
        mShifter.set(!wantsHighGear);
    }
  }

  public synchronized void setOpenLoop(DriveSignal signal) {
    if (mDriveControlState != DriveControlState.OPEN_LOOP) {
        setBrakeMode(true);
        System.out.println("switching to open loop");
        System.out.println(signal);
        mDriveControlState = DriveControlState.OPEN_LOOP;
    }

    mPeriodicIO.left_demand = signal.getLeft();
    mPeriodicIO.right_demand = signal.getRight();
    mPeriodicIO.left_feedforward = 0.0;
    mPeriodicIO.right_feedforward = 0.0;
  }

  private void setDriveLimits(int amps) {
      //mRightMaster.setSmartCurrentLimit(amps);
      //mRightSlave.setSmartCurrentLimit(amps);
      //mLeftMaster.setSmartCurrentLimit(amps);
      //mLeftSlave.setSmartCurrentLimit(amps);
    } 

  private void setDriveCurrentState(DriveCurrentLimitState desiredState, double timestamp) {
    if (desiredState != mDriveCurrentLimitState && (timestamp - mLastDriveCurrentSwitchTime > 1.0)) {
        mLastDriveCurrentSwitchTime = timestamp;
        System.out.println("Switching drive current limit state: " + desiredState);
        if (desiredState == DriveCurrentLimitState.THROTTLED) {
            setDriveLimits(Constants.kDriveCurrentThrottledLimit);
        } else {
            setDriveLimits(Constants.kDriveCurrentUnThrottledLimit);
        }
        mDriveCurrentLimitState = desiredState;
    }
  }

  private void handleFaults() {
  /*if (mRightSlave.getStickyFault(CANSparkMax.FaultID.kHasReset)) {
        System.out.println("Right Slave Reset!");
        mRightSlave.follow(mRightMaster);
        mRightSlave.clearFaults();
    }
    if (mLeftSlave.getStickyFault(CANSparkMax.FaultID.kHasReset)) {
        System.out.println("Left Slave Reset!");
        mLeftSlave.follow(mLeftMaster);
        mLeftSlave.clearFaults();
    }*/
  }

  private void updatePathFollower(double timestamp) {
    if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
        RobotState robot_state = RobotState.getInstance();
        Pose2d field_to_vehicle = robot_state.getLatestFieldToVehicle().getValue();
        Twist2d command = mPathFollower.update(timestamp, field_to_vehicle, robot_state.getDistanceDriven(),
                robot_state.getPredictedVelocity().dx);
        if (!mPathFollower.isFinished()) {
            DriveSignal setpoint = Kinematics.inverseKinematics(command);
            setVelocity(setpoint, new DriveSignal(0, 0));
        } else {
            if (!mPathFollower.isForceFinished()) {
                setVelocity(new DriveSignal(0, 0), new DriveSignal(0, 0));
            }
        }
    } else {
        DriverStation.reportError("drive is not in path following state", false);
    }
  }

  public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
    if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
        setBrakeMode(true);
        System.out.println("switching to path following");
        mDriveControlState = DriveControlState.PATH_FOLLOWING;
        // mLeftMaster.selectProfileSlot(kLowGearVelocityControlSlot, 0);
        // mRightMaster.selectProfileSlot(kLowGearVelocityControlSlot, 0);
        // mLeftMaster.configNeutralDeadband(0.0, 0);
        // mRightMaster.configNeutralDeadband(0.0, 0);
    }
    mPeriodicIO.left_demand = signal.getLeft();
    mPeriodicIO.right_demand = signal.getRight();
    mPeriodicIO.left_feedforward = feedforward.getLeft();
    mPeriodicIO.right_feedforward = feedforward.getRight();
  }

  public synchronized void setHeading(Rotation2d heading) {
    System.out.println("set heading: " + heading.getDegrees());
    
    // mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(mPigeon.getFusedHeading()).inverse());
    mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(mNavx.getRawYawDegrees()).inverse());

    System.out.println("gyro offset: " + mGyroOffset.getDegrees());

    mPeriodicIO.gyro_heading = heading;
}

  public synchronized void setBrakeMode(boolean shouldEnable) {
    if (mIsBrakeMode != shouldEnable) {
        mIsBrakeMode = shouldEnable;
        IdleMode mode = shouldEnable ? IdleMode.kBrake : IdleMode.kCoast;
        //mRightMaster.setIdleMode(mode);
        //mRightSlave.setIdleMode(mode);

        //mLeftMaster.setIdleMode(mode);
        //mLeftSlave.setIdleMode(mode);
    }
  }

  public enum DriveControlState {
    OPEN_LOOP, // open loop voltage control
    PATH_FOLLOWING, // velocity PID control
  }

  public enum DriveCurrentLimitState {
    UNTHROTTLED, THROTTLED
  }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  /*
  @Override
  public void periodic() {
    readPeriodicInputs();
    onLoop(mPeriodicIO.timestamp);
    writePeriodicOutputs();
  }*/
}
