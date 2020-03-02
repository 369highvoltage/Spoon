/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EncoderSubsystem;

public class DriveForward extends CommandBase {
  // DriveSubsystem drive_subsystem;
  // EncoderSubsystem encoder_subsysem;
  double initialPosition;
  double currentPosition;
  double distance;
  double mod = 0.5;


  public DriveForward(double distanceToTravel) {
  //   drive_subsystem = subsystem;
  //   encoder_subsysem = subsystem2;
    // addRequirements(drive_subsystem);
    // addRequirements(encoder_subsysem);
    distance = -distanceToTravel;
    // System.out.println("dist " + distance);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    // encoder_subsysem.resetDriveEncoder();
    initialPosition = RobotContainer.m_encoder_subsystem.getPosition();
    RobotContainer.m_drive_subsystem.tankDrive(1.0, 1.0, mod);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {    
    currentPosition = RobotContainer.m_encoder_subsystem.getPosition();
    // System.out.println("current pos" + currentPosition);
    RobotContainer.m_drive_subsystem.tankDrive(1.0, 1.0, mod);
    /*
    if ((currentPosition - initialPosition) == 0){
      drive_subsystem.tankDrive(1.0, 1.0, mod);
    }
    else if((currentPosition - initialPosition) < distance){
      drive_subsystem.tankDrive(1.0, 1.0, mod);
    }
    */
    // System.out.println("distance" + (currentPosition-initialPosition));
    
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    // System.out.println("end");
    RobotContainer.m_drive_subsystem.tankDrive(0.0, 0.0, mod);
  }
  
  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    // System.out.println("is finished");
    return ((currentPosition - initialPosition) <= distance);
  }
}
