/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.drivers.IO.XboxController;

import lib.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Mechanisms;
import frc.robot.commands.CheesyDrive;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Controllers
  public final XboxController mController = new XboxController(Constants.JoystickConstants.kJoystickPort);

  // Subsystems
  private final Drive mDrive = new Drive();
  
  // Commands 
  private final CheesyDrive mCheesyDrive = new CheesyDrive(mDrive);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    //CommandScheduler.getInstance().setDefaultCommand(mDrive, exampleCommand);

    mDrive.setHeading(Rotation2d.identity());
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //new JoystickButton(m_controller, 5).whenPressed(new Crossbow(mechanisms, 0.8, 1000));
    CheesyDrive.ThrottleInput = () -> mController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.Y);
    CheesyDrive.WheelInput = () -> mController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.Y);
    CheesyDrive.QuickTurnInput = () -> mController.getButton(XboxController.Button.A);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
  }
  */
}
