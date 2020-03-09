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
  public Joystick joyDrive;
  
  public OI() {
    joy = new Joystick(0); //assigns joy to a joystick
    joyDrive = new Joystick(1);
  }

  public Joystick getController() {
    return joy;
  }

  //all axes 

  public double getLeftStick(){ //gets the value of the axis, inverted so forward is 1
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

  public double square(){ 
    if (joy.getRawButton(1)){
      return 1.0;
    } else
    return 0.0;
  }

  public double x(){ //intake
    if (joy.getRawButton(2)){
      return 1.0;
    } else
    return 0.0;
  }

  public boolean circle(){ //autoaim
    return joy.getRawButton(3);
  }
  public boolean isCircleUp(){
    return joy.getRawButtonReleased(3);
  }

  public boolean triangle(){ //turret encoder reset
    return joy.getRawButtonPressed(4);
  }

  public boolean l1(){ //shooting command
    return joy.getRawButton(5);
  }

  public boolean r1(){ 
    return joy.getRawButtonPressed(6);
  }

  public boolean l2(){ 
    return joy.getRawButtonPressed(7);
  }

  public boolean r2(){ 
    return joy.getRawButtonPressed(8);
  }

  public boolean share(){ 
    return joy.getRawButton(9);
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

  public boolean home(){ 
    return joy.getRawButtonPressed(13);
  }

  public boolean rectangleBoi(){
    return joy.getRawButtonPressed(14);
  }

  //drive joystick

  public double driveGetLeftStick(){ 
    return joyDrive.getRawAxis(1);
  }

  public double driveGetRightStick(){ 
    return joyDrive.getRawAxis(5);
  }
  public double driveL1(){
    if (joyDrive.getRawButton(5) == true){
      return 1;
    }else{
      return 0;
    }
    
  }
  public double driveR1(){
    if (joyDrive.getRawButton(6) == true){
      return 1;
    }else{
      return 0;
    }
  }
}
