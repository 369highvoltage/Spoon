/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.sensors.PigeonIMU;  //Pigeon
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*; //TalonSRX
import com.revrobotics.CANEncoder;

public class DriveSubsystem extends SubsystemBase { //create variables are here
  private static final int deviceID1 = 1;
  private static final int deviceID2 = 2;
  private static final int deviceID3 = 3;
  private static final int deviceID4 = 4;
  SpeedController m_frontLeft;
  SpeedController m_frontRight;
  SpeedController m_backLeft;
  SpeedController m_backRight;
  SpeedControllerGroup m_left;
  SpeedControllerGroup m_right;
  DifferentialDrive m_drive;
  PigeonIMU pigeon;
  TalonSRX m_pigeon;
  double[] ypr;
  CANEncoder e_frontLeft;
  CANEncoder e_frontRight;

  public DriveSubsystem(){ //define variables here
    m_frontLeft = new CANSparkMax(deviceID4, MotorType.kBrushless);
    m_frontRight = new CANSparkMax(deviceID1, MotorType.kBrushless);
    m_backLeft = new CANSparkMax(deviceID3, MotorType.kBrushless);
    m_backRight = new CANSparkMax(deviceID2, MotorType.kBrushless);
    m_left = new SpeedControllerGroup(m_frontLeft, m_backLeft);
    m_right = new SpeedControllerGroup(m_frontRight , m_backRight);
    m_drive = new DifferentialDrive(m_left, m_right);
    m_pigeon = new WPI_TalonSRX(5);
    pigeon = new PigeonIMU(m_pigeon); //the pigeon is connected to the TalonSRX(5)
    ypr = new double[3];
  }
  
  public void tankDrive(double leftSpeed, double rightSpeed, double modifier){
    m_drive.tankDrive(leftSpeed*modifier, rightSpeed*modifier);
  }

  public double getYaw(){
    pigeon.getYawPitchRoll(ypr);
    return ypr[0];
  }  

  public void leftDriveEncoder(){
    System.out.println("Drive left enc = " + e_frontLeft.getPositionConversionFactor());
  }

  public void rightDriveEncoder(){
    System.out.println("Drive right enc = " + e_frontRight.getPositionConversionFactor());
  }
  
}
