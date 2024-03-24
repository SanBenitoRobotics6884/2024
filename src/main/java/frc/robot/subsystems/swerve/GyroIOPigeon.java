package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

public class GyroIOPigeon implements GyroIO {
  private Pigeon2 m_gyro;

  private StatusSignal<Double> m_yaw;
  private StatusSignal<Double> m_pitch;
  private StatusSignal<Double> m_roll;

  public GyroIOPigeon(int id) {
    m_gyro = new Pigeon2(id);

    m_yaw = m_gyro.getYaw();
    m_pitch = m_gyro.getPitch();
    m_roll = m_gyro.getRoll();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.yaw = m_yaw.getValueAsDouble();
    inputs.pitch = m_pitch.getValueAsDouble();
    inputs.roll = m_roll.getValueAsDouble();
  }

  @Override
  public void setYaw(double yaw) {
    m_gyro.setYaw(yaw);
  }

}