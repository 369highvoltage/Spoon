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
    public Command autonomous1() {
        System.out.println("autonomus1");
        return new SequentialCommandGroup(
            new DriveForward(2),
            new AutoShooting(0.8, 14000, 1.5),
            new IntakeCommand(2),
            new AutoShooting(0.8, 14000, 1.5),
            new TurnLeft(35, 1),
            new DriveForward(5),
            new TurnRight(-48,1),
            new ParallelCommandGroup(
                new DriveForward(9),
                new IntakeCommand(5)),
            new AutoShooting(0.8, 14000, 2)
        );
    }

    public Command autonomous2() {
        System.out.println("autonomus2");
        return new ParallelCommandGroup(
            new DriveForward(4),
            new IntakeCommand(4)
            
        );
    }
}

