package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends PIDSubsystem {
  private final Spark mShooterMotor = new Spark(ShooterConstants.kShooterPort);

   private final Encoder mShooterEncoder =
      new Encoder(ShooterConstants.kEncoderPort[0], ShooterConstants.kEncoderPort[1], ShooterConstants.kEncoderPort[2]);
  
  /**
   * The shooter subsystem for the robot.
   */
  public Shooter() {
    super(new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD));
    getController().setTolerance(ShooterConstants.kShooterToleranceRPS);
    mShooterEncoder.setDistancePerPulse(ShooterConstants.kEncoderDistancePerPulse);
  }

  @Override
  public void useOutput(double output) {
    mShooterMotor.set(output);
  }

  @Override
  public double getSetpoint() {
    return ShooterConstants.kShooterTargetRPS;
  }

  @Override
  public double getMeasurement() {
    return mShooterEncoder.getRate();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  public void runFeeder() {
    //mFeederMotor.set(ShooterConstants.kFeederSpeed);
  }

  public void stopFeeder() {
   // mFeederMotor.set(0);
  }
}