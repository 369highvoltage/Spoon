/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EncoderSubsystem;

public class DriveForward extends CommandBase {
  DriveSubsystem drive_subsystem;
  EncoderSubsystem encoder_subsysem;
  double initialPosition;
  double currentPosition;
  double distance;


  public DriveForward(DriveSubsystem subsystem, EncoderSubsystem subsystem2, double distanceToTravel) {
    drive_subsystem = subsystem;
    encoder_subsysem = subsystem2;
    addRequirements(drive_subsystem);
    addRequirements(encoder_subsysem);
    distance = distanceToTravel;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    // initialPosition = encoder_subsysem.getPosition();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    // currentPosition = encoder_subsysem.getPosition();
    if (currentPosition - initialPosition == 0){
      drive_subsystem.tankDrive(1.0, 1.0, 1.0);
    }
    else if(currentPosition - initialPosition < distance){
      drive_subsystem.tankDrive(1.0, 1.0, 1.0);
    }
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    drive_subsystem.tankDrive(0.0, 0.0, 0.0);
  }
  
  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return (currentPosition - initialPosition >= distance);
  }
}
