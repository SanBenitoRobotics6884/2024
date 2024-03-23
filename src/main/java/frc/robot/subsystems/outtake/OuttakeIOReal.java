package frc.robot.subsystems.outtake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;

import static frc.robot.Constants.Outtake.*;

public class OuttakeIOReal implements OuttakeIO {
  private TalonFX m_passOffMotor = new TalonFX(PASS_OFF_MOTOR_ID);
  private CANSparkMax m_pivotMotor = new CANSparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);

  private RelativeEncoder m_pivotEncoder = m_pivotMotor.getEncoder();

  private DigitalInput m_speakerLimitSwitch = new DigitalInput(AMP_LIMIT_SWITCH_CHANNEL);

  public OuttakeIOReal() {
    m_pivotMotor.restoreFactoryDefaults();

    CurrentLimitsConfigs config = new CurrentLimitsConfigs();
    config.StatorCurrentLimit = PASS_OFF_CURRENT_LIMIT;
    config.StatorCurrentLimitEnable = true;

    m_passOffMotor.getConfigurator().apply(config);

    m_pivotMotor.setSmartCurrentLimit(PIVOT_CURRENT_LIMIT);
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    inputs.passOffVelocity = m_passOffMotor.getVelocity().getValueAsDouble();
    inputs.passOffCurrent = m_passOffMotor.getStatorCurrent().getValueAsDouble();
    inputs.passOffTemperature = m_passOffMotor.getDeviceTemp().getValueAsDouble();

    inputs.pivotPosition = m_pivotEncoder.getPosition();
    inputs.pivotVelocity = m_pivotEncoder.getVelocity();
    inputs.pivotCurrent = m_pivotMotor.getOutputCurrent();
    inputs.pivotTemperature = m_pivotMotor.getMotorTemperature();

    inputs.speakerLimitSwitch = m_speakerLimitSwitch.get();
  }

  @Override
  public void setPassOffDutyCycle(double percent) {
    m_passOffMotor.set(percent);
  }

  @Override
  public void setPivotDutyCycle(double percent) {
    m_pivotMotor.set(percent);
  }

  @Override
  public void stopPassOffMotor() {
    m_passOffMotor.stopMotor();
  }

  @Override
  public void setPivotPosition(double position) {
    m_pivotEncoder.setPosition(position);
  }

}
