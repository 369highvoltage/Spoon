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

/**
 * Add your docs here.
 */
public class IntakeSubsystem extends SubsystemBase {
  VictorSPX m_intake;
  TalonSRX m_floor;
  double modifyer = 0.25;


  
  public IntakeSubsystem(){
    m_intake = new VictorSPX(9);
    m_floor = new TalonSRX(5); //pigeon in drive subsystem
  }

  public void setIntakeSpeed(double speed){ //sets and regulates intake speed
    if (speed > 1) {
      speed = 1;
    }
    else if (speed < -1) {
      speed = -1;
    }
    m_intake.set(ControlMode.PercentOutput, speed); 
  }

  public void setFloorSpeed(double speed){
    if (speed > 1) {
        speed = 1;
      }
    else if (speed < -1) {
        speed = -1;
      }
    m_floor.set(ControlMode.PercentOutput, speed* modifyer);
  }
}
