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
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.can.*; //Talon FX
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode; //Neutral mode for the Falcons
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;




public class AutoShooting extends CommandBase {
  Timer timer;
  boolean buttonPressed;
  double mod;
  double maximum = 17300;
  double acc;
  double currentTime;
  double runTime;
  Limelight turret_Limelight;

  // TurretSubsystem turret_subsystem;
  // IntakeSubsystem intake_subsystem;
  OI oi;


  
  public AutoShooting(double modifier, double accuracy, double time) {
    // System.out.println("constr ");
    // if the button is pressed the command runs, modifier is used to regulate the speed of the shooter for now
    // turret_subsystem = subsystem;
    // oi = subsystem2;
    // addRequirements(subsystem);
    // addRequirements(subsystem2);
    timer = new Timer();
    turret_Limelight = new Limelight("limelight-turret");
    // turret_subsystem = new TurretSubsystem();
    // intake_subsystem = new IntakeSubsystem();
    // oi = new OI();
    mod = modifier;
    acc = accuracy;
    runTime = time;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    RobotContainer.m_turret_subsystem.shooter(1.0, mod);
    double adjust = turret_Limelight.steeringAdjust();//if there is a target, get the distance from it
    RobotContainer.m_turret_subsystem.setTurretSpeed(-adjust, 0.25);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    double adjust = turret_Limelight.steeringAdjust();//if there is a target, get the distance from it
    RobotContainer.m_turret_subsystem.setTurretSpeed(-adjust, 0.25);
        
    if (RobotContainer.m_turret_subsystem.shooterEncoder() >= acc) {//Once at that speed, fire/load balls
        //17300 for
        //System.out.println("Execute shooter stuff");
        RobotContainer.m_turret_subsystem.shooter(1.0,mod);
        RobotContainer.m_turret_subsystem.feeder(1.0);
        RobotContainer.m_intake_subsystem.setFloorSpeed(-1.0);}
      else{
        RobotContainer.m_turret_subsystem.shooter(1.0,mod);//Charges falcon motors until they reach certain speed
        timer.start();//Starts the timer
      }

      currentTime = timer.get();
      
  }


  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    // System.out.println(" end");
    //Stops all motors and resets timer
    RobotContainer.m_turret_subsystem.shooter(0.0, mod);
    RobotContainer.m_turret_subsystem.feeder(0.0);
    RobotContainer.m_intake_subsystem.setFloorSpeed(0.0);
    timer.reset();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    // System.out.println(" is finished");    
    return (currentTime >= runTime);
    }
  }

