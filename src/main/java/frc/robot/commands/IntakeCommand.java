/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Timer;

public class IntakeCommand extends CommandBase {
  Timer timer;
  double runTime;
  double currentTime;

  public IntakeCommand(double time) {
    timer = new Timer();
    runTime = time;
    
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    timer.start();
    RobotContainer.m_intake_subsystem.setIntakeSpeed(-1.0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    currentTime = timer.get();
    RobotContainer.m_intake_subsystem.setIntakeSpeed(-1.0);
  }

  

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_intake_subsystem.setIntakeSpeed(0.0);
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return (currentTime >= runTime);
  }
 
}
