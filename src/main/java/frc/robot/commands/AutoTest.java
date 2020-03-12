// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.RobotContainer;

/**
 * Add your docs here.
 */
public class AutoTest { //groups of autonomus commands that do different tasks
    public Command autonomous1() { //6 lemon auto right side of the field
        System.out.println("autonomous1");
        return new SequentialCommandGroup(
            new DriveForward(2, 0.75), //drops the intake
            new ParallelCommandGroup( //shoots 3 lemons
                new AutoShooting(0.85, 14000, 2),
                new IntakeCommand(2)),
            new TurnLeft(35, 1), //turns to the trench
            new DriveForward(5.5, 0.75),
            new TurnRight(-49,1),
            new ParallelCommandGroup( //collects 2 lemons from the trench
                new IntakeCommand(4),
                new DriveForward(6, 0.3)
                ),
            new ParallelCommandGroup( //shoot 2 lemons from the trench
                new AutoShooting(0.9 , 14000, 2),
                new IntakeCommand(2)            
            )            
        );
    }

    public Command autonomous2() { //start in the middle of the field
        System.out.println("autonomus2");
        return new SequentialCommandGroup(
            new DriveForward(2, 0.75), //drops the intake //yeet
            new ParallelCommandGroup( //shoots 3 lemons
                new AutoShooting(0.8, 14000, 2),
                new IntakeCommand(2)),
            new DriveForward(3, 0.75),//drives back to the randezvous point 
            new ParallelCommandGroup( //collent lemons
                new DriveForward(2.6, 0.3),
                new IntakeCommand(2)),
            new ParallelCommandGroup( //shoots leoms
                new AutoShooting(0.9, 14000, 3),
                new IntakeCommand(3)
            ));
    }

    // public Command autonomous3(){
    //     return new SequentialCommandGroup(
    //       new DriveForward(2, 0.75),
    //       new   
    //     );
    // }
}

