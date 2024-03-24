package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {
  
  @AutoLog
  public static class OuttakeIOInputs {
    public double passOffVelocity = 0;
    public double passOffCurrent = 0;
    public double passOffTemperature = 0;

    public double pivotPosition = 0;
    public double pivotVelocity = 0;
    public double pivotCurrent = 0;
    public double pivotTemperature = 0;

    public boolean speakerLimitSwitch = false;
  }

  public void updateInputs(OuttakeIOInputs inputs);

  public void setPassOffDutyCycle(double percent);

  public void setPivotDutyCycle(double percent);

  public void stopPassOffMotor();

  public void setPivotPosition(double position);

}
