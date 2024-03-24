package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;

import static frc.robot.Constants.Intake.*;

public class IntakeIOReal implements IntakeIO {

  private final CANSparkMax m_intakeMotor = new CANSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax m_pivotMotor = new CANSparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);

  private DigitalInput m_noteLimitSwitch = new DigitalInput(NOTE_LIMIT_SWITCH);
  private DigitalInput m_zeroLimitSwitch = new DigitalInput(ZERO_LIMIT_SWITCH);

  private RelativeEncoder m_pivotEncoder;
  private RelativeEncoder m_intakeEncoder;

  public IntakeIOReal() {
    m_intakeMotor.restoreFactoryDefaults();
    m_pivotMotor.restoreFactoryDefaults();

    // m_pivotMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    // m_pivotMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_pivotMotor.setSoftLimit(SoftLimitDirection.kForward, (float) DEPLOY_SETPOINT);
    m_pivotMotor.setSoftLimit(SoftLimitDirection.kReverse,(float) STOW_SETPOINT);
    m_pivotMotor.setSmartCurrentLimit(PIVOT_CURRENT_LIMIT);
    m_intakeMotor.setSmartCurrentLimit(INTAKE_CURRENT_LIMIT);
    m_intakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_pivotMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    m_pivotEncoder = m_pivotMotor.getEncoder();
    m_intakeEncoder = m_intakeMotor.getEncoder();
    m_pivotEncoder.setPosition(ENCODER_POSITION);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeVelocity = m_intakeEncoder.getVelocity();
    inputs.intakeCurrent = m_intakeMotor.getOutputCurrent();
    inputs.intakeTemperature = m_intakeMotor.getMotorTemperature();

    inputs.pivotPosition = m_pivotEncoder.getPosition();
    inputs.pivotVelocity = m_pivotEncoder.getVelocity();
    inputs.pivotCurrent = m_pivotMotor.getOutputCurrent();
    inputs.pivotTemperature = m_pivotMotor.getMotorTemperature();

    inputs.noteLimitSwitch = m_noteLimitSwitch.get();
    inputs.zeroLimitSwitch = m_zeroLimitSwitch.get();
  }
  
  @Override
  public void setIntakeDutyCycle(double percent) {
    m_intakeMotor.set(percent);
  }
  @Override
  public void setPivotDutyCycle(double percent) {
    m_pivotMotor.set(percent);
  }
  @Override
  public void stopIntakeMotor() {
    m_intakeMotor.stopMotor();
  }
  @Override
  public void setPivotPosition(double position) {
    m_pivotEncoder.setPosition(position);
  }
  
}
