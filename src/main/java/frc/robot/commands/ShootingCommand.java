/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
//this command enaables the feeder and then the shooter in order to shoot them lemons, aim first
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

import com.ctre.phoenix.motorcontrol.can.*; //Talon FX
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode; //Neutral mode for the Falcons
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj.Timer;



public class ShootingCommand extends CommandBase {
  boolean buttonPressed = true;
  double modifier;
  TurretSubsystem turret_subsystem;

  
  public ShootingCommand(TurretSubsystem subsystem, boolean buttonPressed, double modifier) {
    // if the button is pressed the command runs, modifier is used to regulate the speed of the shooter for now
    turret_subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    turret_subsystem.shooter(1.0, modifier);
    System.out.println("poop");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
   if(turret_subsystem.shooterEncoder() >= 17300){
    turret_subsystem.shooter(1.0, modifier);
    turret_subsystem.feeder(1.0); }
    else {
      turret_subsystem.shooter(1.0, modifier);
    }
  }


  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    turret_subsystem.shooter(0.0, modifier);
    turret_subsystem.feeder(0.0);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return (!buttonPressed);
  }

}