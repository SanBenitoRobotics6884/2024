package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  
  @AutoLog
  public static class GyroIOInputs {
    public double yaw = 0;
    public double pitch = 0;
    public double roll = 0;
  }

  public void updateInputs(GyroIOInputs inputs);

  public void setYaw(double yaw);

}
