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

public class AutoAimCommand extends CommandBase {
  Limelight limelight_subsystem;
  double adjust;
  boolean buttonPressed;

  public AutoAimCommand(Limelight subsystem) {
    System.out.println("autoaim constr");
    addRequirements(subsystem);
    limelight_subsystem = subsystem;
    
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    System.out.println("autoaim init");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    buttonPressed = RobotContainer.m_oi.circle();
    System.out.println("autoaim exec ");
    adjust = limelight_subsystem.steeringAdjust();
    RobotContainer.m_turret_subsystem.setTurretSpeed(-adjust, 0.25);
  }


  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    System.out.println("autoaim end");
    RobotContainer.m_turret_subsystem.setTurretSpeed(0.0, 0.0);
  }

  
  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    if(buttonPressed==false) {
      //if(buttonPressed==false){//if there is no more input, stop shooting
        // System.out.println("Disabled");
        return true;
      }else{
        
      return (!buttonPressed);//returns false if button is pressed
      }
  }

}
