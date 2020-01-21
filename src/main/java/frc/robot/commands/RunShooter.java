/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Shooter;

/**
 * Command: RunShooter
 * Subsystem(s): Shooter
 * Periodic: Yes
 */
public class RunShooter extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  //Controller bindings, to be set with RobotContainer

  private final Shooter shooter;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunShooter(Shooter subsystem) {
    shooter = subsystem;
    addRequirements(subsystem);
    subsystem.setDefaultCommand(this);
  }

  @Override
  public void execute(){
    shooter.enable();
  }

}