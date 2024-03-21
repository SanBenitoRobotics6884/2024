package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimbIOSparkMax implements ClimbIO {
    private final CANSparkMax leader = new CANSparkMax(0, MotorType.kBrushless);
    private final RelativeEncoder encoder = leader.getEncoder();
    

    
}
