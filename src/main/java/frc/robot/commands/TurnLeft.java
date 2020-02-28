/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;

public class TurnLeft extends CommandBase {
  double pigeonVal;
  double pigeonValnit;
  double mod = 0.5;

  
  private final DriveSubsystem drive_subsystem;

  public TurnLeft(DriveSubsystem subsystem) {
    
    drive_subsystem = subsystem;
    addRequirements(drive_subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //get value from pigeon
    pigeonValnit = drive_subsystem.getYaw();
    drive_subsystem.tankDrive(1.0, -1.0, mod);  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pigeonVal= drive_subsystem.getYaw();
    drive_subsystem.tankDrive(1.0, -1.0, mod);   

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive_subsystem.tankDrive(0.0, 0.0, mod);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (pigeonVal > (pigeonValnit + 90));
  }
}