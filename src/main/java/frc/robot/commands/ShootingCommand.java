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
import frc.robot.subsystems.IntakeSubsystem;

import com.ctre.phoenix.motorcontrol.can.*; //Talon FX
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode; //Neutral mode for the Falcons
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;



public class ShootingCommand extends CommandBase {
  Timer timer;
  boolean buttonPressed;
  double mod;
  double maximum = 17300;
  double acc;

  TurretSubsystem turret_subsystem;
  IntakeSubsystem intake_subsystem;
  OI oi;


  
  public ShootingCommand(TurretSubsystem subsystem, OI subsystem2, double modifier, double accuracy) {
    // if the button is pressed the command runs, modifier is used to regulate the speed of the shooter for now
    turret_subsystem = subsystem;
    oi = subsystem2;
    addRequirements(subsystem);
    addRequirements(subsystem2);
    timer = new Timer();
    turret_subsystem = new TurretSubsystem();
    intake_subsystem = new IntakeSubsystem();
    oi = new OI();
    mod = modifier;
    acc = accuracy;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    turret_subsystem.shooter(1.0, mod);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    
    if (turret_subsystem.shooterEncoder() >= acc) {//Once at that speed, fire/load balls
      //17300 for
      //System.out.println("Execute shooter stuff");
      turret_subsystem.shooter(1.0,mod);
      turret_subsystem.feeder(1.0);
      intake_subsystem.setFloorSpeed(-1.0);
      System.out.println("Shooting "+timer.get());
    } else{
      turret_subsystem.shooter(1.0,mod);//Charges falcon motors until they reach certain speed
      SmartDashboard.putNumber("Shooter Speed", turret_subsystem.shooterEncoder());
      timer.start();//Starts the timer
      }
      buttonPressed = oi.l1();
  }


  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    //Stops all motors and resets timer
    turret_subsystem.shooter(0.0, mod);
    turret_subsystem.feeder(0.0);
    intake_subsystem.setFloorSpeed(0.0);
    timer.reset();
    System.out.println("Ended");
    SmartDashboard.putBoolean("Shooting", false);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    if(buttonPressed==false) {
    //if(buttonPressed==false){//if there is no more input, stop shooting
      System.out.println("Disabled");
      return true;
    }else{
      SmartDashboard.putBoolean("Shooting", buttonPressed);//Displays shooting status
    return (!buttonPressed);//returns false if button is pressed
    }
  }

}