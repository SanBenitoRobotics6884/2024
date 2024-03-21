package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    public static class ClimbIOInputs {
        public double leftposition = 0.0;
        public double rigthpostiton = 0.0; 
        public double leftvoltage = 0.0;
        public double rightvoltage = 0.0;
    }

public default void updateInputs(ClimbIOInputs Inputs) {}

public default void setAppliedVoltage(double volts) {}

public default void stop() {}


public default void climb(){}



}