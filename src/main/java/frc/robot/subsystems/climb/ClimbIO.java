package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    public static class ClimbIOInputs {
        public double leftposition;
        public double rightposition; 
        public double leftvoltage = 0.0;
        public double rightvoltage = 0.0;
        public double m_leftCurrent = 0.0; 
        public double m_rightCurrent = 0.0;
        
    }
    public default void updateInputs(ClimbIOInputs Inputs){}
    
    public default void setleftAppliedVoltage(double volts) {}
    public default void setrightAppliedVoltage(double volts){} 
    
    public default void stop() {}
    
    public default void climb(){
    
    }


}