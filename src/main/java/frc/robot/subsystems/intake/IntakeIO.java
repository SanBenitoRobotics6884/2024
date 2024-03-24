package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double intakeVelocity = 0;
    public double intakeCurrent = 0;
    public double intakeTemperature = 0;

    public double pivotPosition = 0;
    public double pivotVelocity = 0;
    public double pivotCurrent = 0;
    public double pivotTemperature = 0;

    public boolean zeroLimitSwitch = false;
    public boolean noteLimitSwitch = false;
  }

  public void updateInputs(IntakeIOInputs inputs);

  public void setIntakeDutyCycle(double percent);

  public void setPivotDutyCycle(double percent);

  public void stopIntakeMotor();

  public void setPivotPosition(double position);



}
