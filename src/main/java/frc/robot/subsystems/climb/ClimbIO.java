package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    public static class ClimbIOInputs {
        public double leftPosition = 0.0;
        public double rightPosition = 0.0; 
        public double leftCurrent = 0.0; 
        public double rightCurrent = 0.0;
        public double leftTemperature = 0.0;
        public double rightTemperature = 0.0;
    }
    public default void updateInputs(ClimbIOInputs Inputs){}
    
    public default void setLeftDutyCycle(double volts) {}

    public default void setRightDutyCycle(double volts) {} 

    public default void setLeftPosition(double position) {}

    public default void setRightPosition(double position) {}

    public default void setBrakeMode(boolean brake) {}

}