// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandGroupBase;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.*;
// import frc.robot.commands.*;
// import frc.robot.subsystems.*;

// public class Autonomus1 extends SequentialCommandGroup {
//   //subsystems
//   TurretSubsystem turret_subsystem;
//   DriveSubsystem drive_subsystem;
//   EncoderSubsystem encoder_subsystem;
//   IntakeSubsystem intake_subsystem;
//   //commands
//   DriveBackward drive_backward;
//   DriveForward drive_forward;
//   TurnLeft turn_left;
//   TurnRight turn_right;


//   /**
//    * Add your docs here.
//    */
//   public Autonomus1() {
//     addCommands(new DriveForward(drive_subsystem, encoder_subsystem, 2));
//     addCommands(new DriveBackward(drive_subsystem, encoder_subsystem, 4));
//     addCommands(new TurnLeft(drive_subsystem));
//     addCommands(new InstantCommand(() -> intake_subsystem.setIntakeSpeed(1.0)));
   
//   }
  
// }
