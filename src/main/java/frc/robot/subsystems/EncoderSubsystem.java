
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class EncoderSubsystem extends SubsystemBase {
    private static final int deviceID1 = 4;
    CANSparkMax m_main;
    CANEncoder encoder;
    int countPerRev;
public EncoderSubsystem(){
    //CANEncoder encoder = new CANEncoder(4);
    m_main = new CANSparkMax(deviceID1, MotorType.kBrushless);
    encoder = new CANEncoder(m_main, EncoderType.kHallSensor, countPerRev);
}
public double getPosition(){
    //System.out.println("drive encoder position" + encoder.getPosition()/(-4.87));
    return (double)encoder.getPosition()/(-4.87);
}
public double getVelocity(){
    //System.out.println("drive encoder velocity" + encoder.getVelocity());
    return (double)encoder.getVelocity();
}

}