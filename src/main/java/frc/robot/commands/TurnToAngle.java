/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngle extends CommandBase {
  private final DriveSubsystem drive_subsystem;

  public TurnToAngle(double targetAngle, DriveSubsystem subsystem) {
    super(
      new PIDController(kP, kI, kD),
      drive_subsystem::getYaw,
      targetAngle,
      output -> drive_subsystem.,
      drive_subsystem
    );

    drive_subsystem = subsystem;
    addRequirements(drive_subsystem);
        // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //get value from pigeon
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive_subsystem.tankDrive(0.0, 0.0, 1.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
