/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.ShootingCommand;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.subsystems.*;

public class OI extends SubsystemBase {
  public Joystick joy; //creates joy
  TurretSubsystem turret_subsystem;
  
  public OI() {
    joy = new Joystick(0); //assigns joy to a joystick

  }

  public Joystick getController() {
    return joy;
  }

  //all axes 

  public double getLeftStick(){ 
    return joy.getRawAxis(1);
  }

  public double getLeftTurretAxis(){
    return joy.getRawAxis(3)/2+0.5;
  }

  public double getRightTurretAxis(){
    return joy.getRawAxis(4)/2+0.5;
  }

  public double getRightStick(){ 
    return joy.getRawAxis(5);
  }

  /* all buttons
  getRawButtonPressed for the event and
  getRawButton for the state of the button */

  public boolean square(){ 
    return joy.getRawButtonPressed(1);
  }

  public boolean x(){ 
    return joy.getRawButtonPressed(2);
  }

  public boolean circle(){ //aiming
    return joy.getRawButtonPressed(3);
  }

  public boolean triangle(){ //resetting the encoder
    return joy.getRawButtonPressed(4);
  }

  public boolean l1(){ //shooting command
    return joy.getRawButton(5);
  }

  public double r1(){ 
    if (joy.getRawButton(6) == true){
      return 1.0;
    }
    else {
      return 0.0;
    }
  }

  public boolean l2(){ 
    return joy.getRawButtonPressed(7);
  }

  public boolean r2(){ 
    return joy.getRawButtonPressed(8);
  }

  public boolean share(){ 
    return joy.getRawButtonPressed(9);
  }

  public boolean options(){
    return joy.getRawButtonPressed(10);
  }

  public boolean leftStick(){ 
    return joy.getRawButtonPressed(11);
  }

  public boolean rightStick(){
    return joy.getRawButtonPressed(12);
  }

  public boolean pspsps(){ 
    return joy.getRawButtonPressed(13);
  }

  public boolean bigButtonBoi(){
    return joy.getRawButtonPressed(14);
  }
}
