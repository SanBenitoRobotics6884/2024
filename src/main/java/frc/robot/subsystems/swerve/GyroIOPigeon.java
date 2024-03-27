package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;

public class GyroIOPigeon implements GyroIO {
  private Pigeon2 m_gyro;

  public GyroIOPigeon(int id) {
    m_gyro = new Pigeon2(id);

    m_gyro.optimizeBusUtilization();
    m_gyro.getYaw().setUpdateFrequency(50);
    m_gyro.getPitch().setUpdateFrequency(50);
    m_gyro.getRoll().setUpdateFrequency(50);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.yaw = m_gyro.getYaw().getValueAsDouble();
    inputs.pitch = m_gyro.getPitch().getValueAsDouble();
    inputs.roll = m_gyro.getRoll().getValueAsDouble();
  }

  @Override
  public void setYaw(double yaw) {
    m_gyro.setYaw(yaw);
  }

}