/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
//this command enaables the feeder and then the shooter in order to shoot them lemons, aim first
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.OI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class IntakeCommand extends CommandBase {
  Timer timer;
  boolean buttonPressed;
  Limelight intake_limelight;
  double mod;
  IntakeSubsystem intake_subsystem;
  DriveSubsystem drive_subsystem;
  OI oi;


  
  public IntakeCommand(Limelight subsystem1, OI subsystem2, DriveSubsystem subsystem3, double modifier) {
    // if the button is pressed the command runs, modifier is used to regulate the speed of the shooter for now
    intake_limelight = subsystem1;
    oi = subsystem2;
    drive_subsystem = subsystem3;
    addRequirements(subsystem1);
    addRequirements(subsystem2);
    addRequirements(subsystem3);

    timer = new Timer();
    intake_limelight = new Limelight("limelight-intake");
    intake_subsystem = new IntakeSubsystem();
    drive_subsystem = new DriveSubsystem();
    oi = new OI();
    mod = modifier;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    //intake_limelight.setLEDMode(3);
    intake_limelight.setPipeline(0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    
    if (intake_limelight.canSeeTarget()==true) {
      double leftAdjust = -1.0; 
      double rightAdjust = -1.0; // default speed values for chase

    leftAdjust -= intake_limelight.steeringAdjust();//adjust each side according to tx
    rightAdjust += intake_limelight.steeringAdjust();
/*
     if(Math.abs(intake_limelight.offsetY()) <= mindistance){//checks if the height is less than five, if it is stop 
       drive_subsystem.tankDrive(0, 0, 1);
     }else{
       */
       if(intake_limelight.canSeeTarget() == false){//check if there is target, if not, spin
         drive_subsystem.tankDrive(-.5, .5, .5);
       }else if((intake_limelight.canSeeTarget() == true)){//check if there is target, use adjust values to move
         drive_subsystem.tankDrive(leftAdjust, rightAdjust, 0.5);
         intake_subsystem.setIntakeSpeed(-.5);
         
     }
    }
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    //Stops all motors and resets timer
    intake_limelight.setPipeline(1);
    intake_limelight.setLEDMode(0);
    intake_subsystem.setIntakeSpeed(0.0);
    timer.reset();
    System.out.println("Ended");
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    if(buttonPressed==false) {
    //if(buttonPressed==false){//if there is no more input, stop shooting
      System.out.println("Disabled");
      return true;
    }else{
      SmartDashboard.putBoolean("Following", buttonPressed);//Displays shooting status
    return (!buttonPressed);//returns false if button is pressed
    }
  }

}