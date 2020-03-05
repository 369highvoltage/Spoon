/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.commands.*;

public class AutonomousV1 {
  AutoTest m_autoTest;
  /**
   * Add your docs here.
   */
  public AutonomousV1(){
    m_autoTest = new AutoTest();
  }


  public Command AutonomousV1() {
    return new SequentialCommandGroup(
      m_autoTest.autonomous1(),
      m_autoTest.autonomous2()
    );
   
  }
}
