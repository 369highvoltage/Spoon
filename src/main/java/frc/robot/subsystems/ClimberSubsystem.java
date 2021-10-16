package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    SpeedController climberMotor;

    public ClimberSubsystem() {
        //Define variables here
        climberMotor = new CANSparkMax(5, MotorType.kBrushless);
    }


    public void activateClimber(double input, double modifier) {
        //DO NOT GO ABOVE 1 FOR MODIFIER
        climberMotor.set(input*modifier);
    }

    @Override
    public void periodic() {

    }

}
