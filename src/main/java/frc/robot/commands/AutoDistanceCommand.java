/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.*;
import frc.robot.RobotContainer;

public class AutoDistanceCommand extends CommandBase {
  Limelight limelight_subsystem;
  double adjust;
  boolean button;

  public AutoDistanceCommand(Limelight subsystem, boolean button) {
    System.out.println("Auto Distance constr");
    addRequirements(subsystem);
    limelight_subsystem = subsystem;
    this.button = button;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    System.out.println("Auto Distance init");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    System.out.println("Auto Distance exec ");
    adjust = limelight_subsystem.steeringAdjust();
    RobotContainer.m_turret_subsystem.setTurretSpeed(-adjust, 0.25);
  }


  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    System.out.println("Auto Distance end");
    RobotContainer.m_turret_subsystem.setTurretSpeed(0.0, 0.0);
  }

  
  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    System.out.println("Auto Distance is finished");
    return (button);
  }

}
