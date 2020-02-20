
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class EncoderSubsystem extends SubsystemBase {
    private static final int deviceID4 = 4;
    private static final int deviceID1 = 1;
    CANSparkMax m_left;
    CANSparkMax m_right;
    CANEncoder encoderLeft;
    CANEncoder encoderRight;
    int countPerRev;
public EncoderSubsystem(){
    //CANEncoder encoder = new CANEncoder(4);
    m_left = new CANSparkMax(deviceID4, MotorType.kBrushless);
    encoderLeft = new CANEncoder(m_left, EncoderType.kHallSensor, countPerRev);
    m_right = new CANSparkMax(deviceID1, MotorType.kBrushless);
    encoderRight = new CANEncoder(m_right, EncoderType.kHallSensor, countPerRev);
}
public double getPosition(CANEncoder encoder){
    return (double)encoder.getPositionConversionFactor();
}
public double getVelocity(CANEncoder encoder){
    return (double)encoder.getVelocityConversionFactor();
}
/*
map the input to -180 to 180:

angle = ((((encoder_val - zero_offset) % 4096) + 4096) % 4096 - 2048) * 45 / 512

encoder_val is the raw number back from the encoder (works whether continuous is true or not.)
zero_offset is your -268.

The first modulo (%) pulls into the range -4095…4095, the add and next modulo make it 0…4095, subtracting 2048 makes it -4096…4095, and the multiplication and division scale it to 180.

*/
public double convertAngle(double rotation){
    double angle;//Output variable
    double rot = rotation;//Current value
    double max = 6000;//Maximum value of the encoder
    angle = ((rot % max)+max);
    return angle;
}


}