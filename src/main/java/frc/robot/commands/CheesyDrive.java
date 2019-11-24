/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivers.IO.UserInputDouble;
import frc.robot.drivers.IO.UserInputBoolean;
import frc.robot.subsystems.Drive;

/**
 * Command: CheesyDrive
 * Subsystem(s): Drive
 * Periodic: Yes
 */
public class CheesyDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  //Controller bindings, to be set with RobotContainer

  private final Drive drive;

  public static UserInputDouble ThrottleInput;
  public static UserInputDouble WheelInput;
  public static UserInputBoolean QuickTurnInput;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CheesyDrive(Drive subsystem) {
    drive = subsystem;
    addRequirements(subsystem);
    subsystem.setDefaultCommand(this);
  }

  @Override
  public void execute(){
    drive.setCheesyishDrive(ThrottleInput.getValue(), WheelInput.getValue(), QuickTurnInput.getValue());
  }
}