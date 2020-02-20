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
import com.ctre.phoenix.motorcontrol.ControlMode;
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
  
  public TurretSubsystem(){
    m_feeder = new VictorSPX(10); //pigeon in drive subsystem
    m_turret = new TalonSRX(6);
    m_shooterLeft = new TalonFX(7);
    m_shooterRight = new TalonFX(8);
    boolean _brake = true;
    m_shooterLeft.setNeutralMode(NeutralMode.Brake);
    m_shooterRight.setNeutralMode(NeutralMode.Brake);
  }

  public void setTurretSpeed(double speed, double modifier){ //sets and regulates turret speed
    if (speed > 1) {
      speed = 1;
    }
    else if (speed < -1) {
      speed = -1;
    }

    if (modifier > 1) {
      modifier = 1;
    }
    else if (modifier < 0) {
      modifier = 0;
    }
    m_turret.set(ControlMode.PercentOutput, speed* modifier); 
  }

  public double encoderVal(){ //gets the value of the turret magnetic encoder
    double position = m_turret.getSelectedSensorPosition(0);
    return position;
    //Counter clockwise from hopper is -10000
    //Clockwise from hopper is 10000
    //Limit set at ~8000
  }
  public void encoderReset(boolean button){
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
    return m_shooterRight.getSelectedSensorVelocity();
  }

   public void feeder(double speed){
      m_feeder.set(ControlMode.PercentOutput,speed);
    }

}
