/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
//Commands & Subsystems
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.networktables.NetworkTable;
//Individual imports
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.DriverStation; // Import to get communications from DriverStation

public class Robot extends TimedRobot {  
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  //commands
  DriveForward drive_forward;
  DriveBackward drive_backward;
  TurnLeft turn_left;
  TurnRight turn_right;
  ShootingCommand shooting_command;
  // AutoTest autonomus;
  AutonomousV1 autonomousV1;
  IntakeCommand intake_command;
  //Autonomus1 autonomus1;
  //subsystem
  // DriveSubsystem drive_subsystem;
  Limelight turret_Limelight;
  // EncoderSubsystem encoder_subsystem;
  // OI oi;
  // TurretSubsystem turret_subsystem;
  // IntakeSubsystem intake_subsystem;
  //variables
  double turretVal;
  double turretVal2;
  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightSteerCommand = 0.0;
  JoystickButton btn;
  NetworkTable turnin_pid_table;
  
 

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    // drive_subsystem = new DriveSubsystem();
    //camera_subsystem = new CameraSubsystem();
    // encoder_subsystem = new EncoderSubsystem();
    // turret_subsystem = new TurretSubsystem();
    // intake_subsystem = new IntakeSubsystem();
    turret_Limelight = new Limelight("Turret");
    // turn_left = new TurnLeft();
    // turn_right = new TurnRight();
    // oi = new OI();
    btn = new JoystickButton(RobotContainer.m_oi.getController(), 5);
    // autonomus = new AutoTest();
    
    autonomousV1 = new AutonomousV1();
    turnin_pid_table = NetworkTableInstance.getDefault().getTable("turnin_pid");
    turnin_pid_table.getEntry("kP").setDouble(0.1);
    turnin_pid_table.getEntry("kI").setDouble(0);
    turnin_pid_table.getEntry("kD").setDouble(0);
   


    //settings when the robot turns on
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    switch(DriverStation.getInstance().getLocation()) {
      case 1:
        System.out.println("Left side")
        m_autonomousCommand = new AutoTest().autonomous1();
        break;
      case 2:
        System.out.println("Middle");
        m_autonomousCommand = new AutoTest().autonomous2();
        break;
      case 3:
        System.out.println("Right Side");
        // Case 3 logic here
        break;
      default:
        System.out.println("No Autonomous found");
        break;
    }


    System.out.println("auto init");
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    System.out.println("auto periodic");
    double leftAdjust = -1.0; 
    double rightAdjust = -1.0; // default speed values for chase
    double mindistance = 5;
    leftAdjust -= turret_Limelight.steeringAdjust();//adjust each side according to tx
    rightAdjust += turret_Limelight.steeringAdjust();
/*
     if(Math.abs(camera_subsystem.getTy()) <= mindistance){//checks if the height is less than five, if it is stop 
       drive_subsystem.tankDrive(0, 0, 1);
     }else{
       if(camera_subsystem.isTarget() == false){//check if there is target, if not, spin
         drive_subsystem.tankDrive(-.5, .5, .5);
       }else if((camera_subsystem.isTarget() == true)){//check if there is target, use adjust values to move
         drive_subsystem.tankDrive(leftAdjust, rightAdjust, 0.5);
         }
     }
  }
*/}
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //camera_subsystem.ledOff();
    boolean m_LimelightHasValidTarget;

    btn.whenPressed(new ShootingCommand(0.8, 14000));
    System.out.println("teleop init");
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    System.out.println("teleop periodic");
    RobotContainer.m_drive_subsystem.tankDrive(RobotContainer.m_oi.driveGetLeftStick(), RobotContainer.m_oi.driveGetRightStick(), 0.95);
    RobotContainer.m_drive_subsystem.getYaw();
    turretVal = RobotContainer.m_oi.getLeftTurretAxis();//Get fixed inputs from oi
    turretVal2 = RobotContainer.m_oi.getRightTurretAxis();

    turretVal2 = turretVal-turretVal2;//final calculations
    RobotContainer.m_turret_subsystem.setTurretSpeed(turretVal2, 0.25);

    //Autoaim (toggle)
    if (RobotContainer.m_oi.circle()==true){
      while(RobotContainer.m_oi.circleup()!=true){
        if (turret_Limelight.canSeeTarget()==false){
          //if there is no target, do nothing
        }else if((turret_Limelight.canSeeTarget()==true)){
          double adjust = turret_Limelight.steeringAdjust();//if there is a target, get the distance from it
          RobotContainer.m_turret_subsystem.setTurretSpeed(adjust, 0.25);//set the speed to that distance, left is negative and right is positive
        }
      }
    }

   
    RobotContainer.m_intake_subsystem.setFloorSpeed(-RobotContainer.m_oi.square());
    RobotContainer.m_intake_subsystem.setIntakeSpeed(-RobotContainer.m_oi.x());
    RobotContainer.m_turret_subsystem.encoderVal(); //turret encoder

    if(RobotContainer.m_oi.r1()){
      RobotContainer.m_turret_subsystem.encoderReset();
    }
    System.out.println(RobotContainer.m_turret_subsystem.encoderVal());
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
  //Other functions

  public void print(String value){
    System.out.println(value);
  }
  
}
