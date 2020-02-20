/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveForward;
import frc.robot.commands.TurnLeft;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EncoderSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class DriveFrwdTurnLeft extends SequentialCommandGroup  {
  DriveSubsystem drive_subsystem;
  EncoderSubsystem encoder_subsystem;
  DriveForward drive_forward;
  TurnLeft turn_left;
  
  /**
   * Add your docs here.
   */
  public DriveFrwdTurnLeft(DriveForward drive_forward, TurnLeft turn_left, double distanceToTravel ) {
    drive_subsystem = new DriveSubsystem();
    encoder_subsystem = new EncoderSubsystem();
    addCommands(new DriveForward(drive_subsystem, encoder_subsystem, distanceToTravel));
    addCommands(new TurnLeft(drive_subsystem));
    
  }
}
