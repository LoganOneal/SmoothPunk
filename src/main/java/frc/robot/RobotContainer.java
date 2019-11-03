/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Crossbow;
import frc.robot.commands.PullIntake;
import frc.robot.commands.TestGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Mechanisms;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private final Mechanisms mechanisms = new Mechanisms();
  
  private Joystick m_controller = new Joystick(0);
  private RunCommand c_joystick = new RunCommand(
    () -> drivetrain.driveCartesian(m_controller.getX() * -1,
      m_controller.getY() * -1,
      m_controller.getTwist()), drivetrain);

  private RunCommand c_shooter = new RunCommand(
    () -> mechanisms.setIntake(m_controller.getRawButton(7), m_controller.getRawButton(8)), mechanisms);
  

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    drivetrain.setDefaultCommand(c_joystick);
    mechanisms.setDefaultCommand(c_shooter);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_controller, 5).whenPressed(new Crossbow(mechanisms, 0.8, 1000));
    new JoystickButton(m_controller, 6).whenPressed(new Crossbow(mechanisms, -0.6, 1750));
    new JoystickButton(m_controller, 4).whenPressed(new PullIntake(mechanisms));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new TestGroup(mechanisms, drivetrain, m_controller);
  }
}
