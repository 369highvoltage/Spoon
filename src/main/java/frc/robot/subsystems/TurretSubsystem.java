/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
/**
 * Add your docs here.
 */
public class TurretSubsystem extends SubsystemBase {
  TalonSRX m_turret;
  VictorSPX m_feeder;
  TalonFX m_shooterLeft;
  TalonFX m_shooterRight;
  double turLeftLimit= -7000;
  double turRightLimit = 7000;
  
  public TurretSubsystem(){
    m_feeder = new VictorSPX(10); //pigeon in drive subsystem
    m_turret = new TalonSRX(6);
    m_shooterLeft = new TalonFX(7);
    m_shooterRight = new TalonFX(8);
    // boolean _brake = true;
    m_turret.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    // m_turret
    m_shooterLeft.setNeutralMode(NeutralMode.Brake);
    m_shooterRight.setNeutralMode(NeutralMode.Brake);
    m_feeder.setNeutralMode(NeutralMode.Coast);
  }

  public double encoderVal(){ //gets the raw values of the turret magnetic encoder
    double position = m_turret.getSelectedSensorPosition();
    // SmartDashboard.putString("turret encoder value " ,  m_turret.getSelectedSensorPosition());
    // System.out.println("turret encoder value " +  position);
    return  position;
    //Counter clockwise from hopper is -10000
    //Clockwise from hopper is 10000
    //Limit set at ~8000
  }

  public void setTurretSpeed(double speed, double modifier){ //sets and regulates turret speed
    //speed limits
    if (speed > 1) {
      speed = 1;
    }
    else if (speed < -1) {
      speed = -1;
    }
    //modifier limits
    if (modifier > 1) {
      modifier = 1;
    }
    else if (modifier < 0) {
      modifier = 0;
    }
     //turning limits
    if (encoderVal()<=turLeftLimit){
      m_turret.set(ControlMode.PercentOutput, 0.1); //move to right
    }
    else if(encoderVal()>=turRightLimit){
      m_turret.set(ControlMode.PercentOutput, -0.1); //move to left
    }
    else if (encoderVal()>turLeftLimit && encoderVal()<turRightLimit){
    m_turret.set(ControlMode.PercentOutput, -speed* modifier); 
    }
  }
  
  public void encoderReset(boolean button){
    if(button)
    m_turret.setSelectedSensorPosition(0);
  }

  public void shooter(double speed, double modifier){
    if (speed == 0.0){
      m_shooterLeft.set(TalonFXControlMode.PercentOutput,0.0*modifier);
      m_shooterRight.set(TalonFXControlMode.PercentOutput,0.0*modifier);
    }
    else{
      m_shooterLeft.set(TalonFXControlMode.PercentOutput, -speed*modifier);
      m_shooterRight.set(TalonFXControlMode.PercentOutput, speed*modifier);
    }
  }

  public double shooterEncoder(){
    // System.out.println("shooter encoder"+  m_shooterRight.getSelectedSensorVelocity());
    return m_shooterRight.getSelectedSensorVelocity();
  }

   public void feeder(double speed){
      m_feeder.set(ControlMode.PercentOutput,speed);
    }
}